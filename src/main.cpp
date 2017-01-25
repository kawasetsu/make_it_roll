// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/***************************************************/
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArm, drvGaze;
    ICartesianControl *iarm;
    IGazeControl      *igaze;

    BufferedPort<ImageOf<PixelRgb> > imgLPortIn,imgRPortIn;
    BufferedPort<ImageOf<PixelRgb> > imgLPortOut,imgRPortOut;
    RpcServer rpcPort;

    Mutex mutex;
    Vector cogL,cogR;
    bool okL,okR;

    //context
    int context_init;

    //vector for initial pose
    Vector vectPosInit, vectOriInit, vectGazeInit;

    /***************************************************/
    bool getCOG(ImageOf<PixelRgb> &img, Vector &cog)
    {
        int xMean=0;
        int yMean=0;
        int ct=0;

        for(int x=0; x<img.width(); x++){
            for (int y=0; y<img.height(); y++){
                PixelRgb &pixel=img.pixel(x,y);
                if ((pixel.b>5.0*pixel.r) && (pixel.b>5.0*pixel.g)){
                    xMean+=x;
                    yMean+=y;
                    ct++;
                }
            }
        }

        if (ct>0){
            cog.resize(2);
            cog[0]=xMean/ct;
            cog[1]=yMean/ct;
            return true;
        }
        else{
            return false;
        }
    }

    /***************************************************/
    Vector retrieveTarget3D(const Vector &cogL, const Vector &cogR)
    {   

        // get the projection
        Vector vectPos;
        igaze->triangulate3DPoint(cogL,cogR,vectPos);
        
		return vectPos;
    }

    /***************************************************/
    void fixate(const Vector &x)
    {
        // request to gaze at the desired fixation point and wait
        igaze->lookAtFixationPoint(x);
        igaze->waitMotionDone();
        // get the current fixation point and measure the error
        Vector tmp;
        igaze->getFixationPoint(tmp);
        cout<<"final error = "<<norm(x-tmp)<<endl;
    }

    /***************************************************/
    Vector computeHandOrientation()
    {
        //set the hand orientation
        Matrix R(3,3);
        // pose x-axis   y-axis        z-axis
        R(0,0)=-1.0;  R(0,1)= 0.0;  R(0,2)= 0.0;    // x-coordinate in root frame
        R(1,0)= 0.0;  R(1,1)= 0.0;  R(1,2)=-1.0;    // y-coordinate    "
        R(2,0)= 0.0;  R(2,1)=-1.0;  R(2,2)= 0.0;    // z-coordinate    "
        Vector vectOri=yarp::math::dcm2axis(R);
        
        return vectOri;
    }

    /***************************************************/
    void approachTargetWithHand(const Vector &x, const Vector &o)
    {
        // move to the right hand to the side of the ball
        Vector vectPos = x;
        vectPos[1] += 0.1;

        // send request and wait until the motion is done
        iarm->goToPoseSync(vectPos,o);
        iarm->waitMotionDone(0.04);
    }

    /***************************************************/
    void roll(const Vector &x, const Vector &o)
    {
        // move to the right hand to push the ball
        Vector vectPos = x;
        vectPos[1] -= 0.1;

        //set the operation time
        iarm->setTrajTime(0.2);

        // send request and wait until the motion is done
        iarm->goToPoseSync(vectPos, o);
        iarm->waitMotionDone(0.04);

        //return initial context
        iarm->restoreContext(context_init);
    }

    /***************************************************/
    void look_down()
    {
        // set the position where to look
        Vector fp(3);
        fp[0]=-0.50;    // x-component [m]
        fp[1]=+0.00;    // y-component [m]
        fp[2]=+0.00;    // z-component [m]
        // request to gaze at the desired fixation point and wait
        igaze->lookAtFixationPoint(fp);
        igaze->waitMotionDone();
        // get the current fixation point and measure the error
        Vector x;
        igaze->getFixationPoint(x);
        cout<<"final error = "<<norm(fp-x)<<endl;
    }

    /***************************************************/
    void make_it_roll(const Vector &cogL, const Vector &cogR)
    {
        yInfo()<<"detected cogs = ("<<cogL.toString(0,0)<<") ("<<cogR.toString(0,0)<<")";

        Vector x=retrieveTarget3D(cogL,cogR);
        yInfo()<<"retrieved 3D point = ("<<x.toString(3,3)<<")";

        fixate(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        Vector o=computeHandOrientation();
        yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

        approachTargetWithHand(x,o);
        yInfo()<<"approached";

        roll(x,o);
        yInfo()<<"roll!";
    }

    /***************************************************/
    void home()
    {
        // move to the initial pose
        iarm->goToPoseSync(vectPosInit,vectOriInit);
        iarm->waitMotionDone(0.04);

        // gaze at the initial point
        igaze->lookAtFixationPoint(vectGazeInit);
        igaze->waitMotionDone();
    }

public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        // initialize the right arm controller
        Property optArm;
        optArm.put("device","cartesiancontrollerclient");
        optArm.put("remote","/icubSim/cartesianController/right_arm");
        optArm.put("local","/cartesian_client/right_arm");

        // let's give the controller some time to warm up
        bool ok=false;
        double t0=Time::now();
        while (Time::now()-t0<10.0){
            // this might fail if controller is not connected to solver yet
            if (drvArm.open(optArm)){
                ok=true;
                break;
            }
            Time::delay(1.0);
        }

        if (!ok){
            yError()<<"Unable to open the Cartesian Controller";
            return false;
        }

        if (drvArm.isValid()){
            drvArm.view(iarm);
        }

        //enable the torso joints
        Vector curDof;
        iarm->getDOF(curDof);
        Vector newDof(3);
        newDof[0]=1;    // torso pitch: 1 => enable
        newDof[1]=1;    // torso roll:  1 => enable
        newDof[2]=1;    // torso yaw:   1 => enable
        iarm->setDOF(newDof,curDof);


        // initialize the gaze controller
        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/client/gaze");

        // let's give the controller some time to warm up
        ok=false;
        t0=Time::now();
        while (Time::now()-t0<10.0){
            // this might fail if controller is not connected to solver yet
            if (drvGaze.open(optGaze)){
                ok=true;
                break;
            }
            Time::delay(1.0);
        }

        if (!ok){
            yError()<<"Unable to open the Cartesian Controller";
            return false;
        }

        if (drvGaze.isValid()){
            drvGaze.view(igaze);
        }


        //store the initial pose
        iarm->getPose(vectPosInit, vectOriInit);
        igaze->getFixationPoint(vectGazeInit);


        //store initial context
        iarm->storeContext(&context_init);

        //set the operation time
        iarm->setTrajTime(1);

        //open the ports
        imgLPortIn.open("/imgL:i");
        imgRPortIn.open("/imgR:i");

        imgLPortOut.open("/imgL:o");
        imgRPortOut.open("/imgR:o");

        rpcPort.open("/service");
        attach(rpcPort);

        return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        imgLPortIn.interrupt();
        imgRPortIn.interrupt();
        return true;
    }

    /***************************************************/
    bool close()
    {
        drvArm.close();
        drvGaze.close();
        imgLPortIn.close();
        imgRPortIn.close();
        imgLPortOut.close();
        imgRPortOut.close();
        rpcPort.close();
        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        if(cmd=="help"){
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- look_down");
            reply.addString("- make_it_roll");
            reply.addString("- home");
            reply.addString("- quit");
        }
        else if(cmd=="look_down"){
            look_down();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yep! I'm looking down now!");
        }
        else if(cmd=="make_it_roll"){
            //check whether the ball is on both views
            mutex.lock();
            bool go = okL && okR;
            mutex.unlock();

            if(go){
                make_it_roll(cogL,cogR);
                // we assume the robot is not moving now
                reply.addString("ack");
                reply.addString("Yeah! I've made it roll like a charm!");
            }
            else{
                reply.addString("nack");
                reply.addString("I don't see any object!");
            }
        }
        else if (cmd=="home"){
            home();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("I've got the hard work done! Going home.");
        }
        else{
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);
        }

        return true;
    }

    /***************************************************/
    double getPeriod()
    {
        // sync upon incoming images
        return 0.0;
    }

    /***************************************************/
    bool updateModule()
    {
        // get fresh images
        ImageOf<PixelRgb> *imgL=imgLPortIn.read();
        ImageOf<PixelRgb> *imgR=imgRPortIn.read();

        // interrupt sequence detected
        if((imgL==NULL) || (imgR==NULL)){
            return false;
        }

        // compute the center-of-mass of pixels of our color
        mutex.lock();
        okL=getCOG(*imgL,cogL);
        okR=getCOG(*imgR,cogR);
        mutex.unlock();

        PixelRgb color;
        color.r=255; color.g=0; color.b=0;

        if (okL){
            draw::addCircle(*imgL,color,(int)cogL[0],(int)cogL[1],5);
        }

        if (okR){
            draw::addCircle(*imgR,color,(int)cogR[0],(int)cogR[1],5);
        }

        imgLPortOut.prepare()=*imgL;
        imgRPortOut.prepare()=*imgR;

        imgLPortOut.write();
        imgRPortOut.write();

        return true;
    }
};


/***************************************************/
int main()
{
    Network yarp;
    if(!yarp.checkNetwork()){
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    CtrlModule mod;
    ResourceFinder rf;
    return mod.runModule(rf);
}
