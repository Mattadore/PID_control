#include <ros/ros.h>
#include <tf/transform_datatypes.h> //idk
#include <PID_control/motoroutputs.h>
#include <PID_control/quadstate.h>

#define throttled false

double pParam=2,iParam=0,dParam=0,vParam=-2,cumul=0,lastE=0,pAlphaE=0,pBetaE=0,psp2=0,psp1=0,prevYaw=0;
double thrust,pv,betaE,alphaE,betaCorr,alphaCorr,yaw,rotCorr,yError,euler,t1,t2,e,a,b,c,d;
tf::Vector3 sp,vx,vy,vel;
tf::Transform pose;
ros::Publisher pub;
PID_control::motoroutputs ret;

void control(const PID_control::quadstate::ConstPtr & req) {
    tf::vector3MsgToTF(req->target_error,sp);
    tf::transformMsgToTF(req->transform,pose);
    tf::vector3MsgToTF(req->velocity,vel);
    if (req->reset) {
        pParam=2,iParam=0,dParam=0,vParam=-2,cumul=0,lastE=0,pAlphaE=0,pBetaE=0,psp2=0,psp1=0,prevYaw=0;
        ROS_INFO("Controller reset");
    }
	e = (pose*sp).z() - pose.getOrigin().z();
	cumul=cumul+e;
    pv=pParam*e;
    thrust=5.335+pv+iParam*cumul+dParam*(e-lastE)+vel.z()*vParam;
    lastE=e;
    // Horizontal control: 
    vx = pose*tf::Vector3(1,0,0);
    vy = pose*tf::Vector3(0,1,0);
    alphaE=(vy.z()-pose.getOrigin().z());
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE);
    betaE=(vx.z()-pose.getOrigin().z());
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE);
    pAlphaE=alphaE;
    pBetaE=betaE;
    alphaCorr=alphaCorr+sp.y()*0.005+1*(sp.y()-psp2);
    betaCorr=betaCorr-sp.x()*0.005-1*(sp.x()-psp1);
    psp2=sp.y();
    psp1=sp.x();
    pose.getBasis().getRPY(t1, t2, yaw);
    rotCorr=yaw*0.1+2*(yaw-prevYaw);
    prevYaw = yaw;
    // Decide of the motor velocities:
    a=thrust*((double)1-alphaCorr+betaCorr+rotCorr);
    b=thrust*((double)1-alphaCorr-betaCorr-rotCorr);
    c=thrust*((double)1+alphaCorr-betaCorr+rotCorr);
    d=thrust*((double)1+alphaCorr+betaCorr-rotCorr);
    ret.a=a;
    ret.b=b;
    ret.c=c;
    ret.d=d;
    pub.publish(ret);
}

int main(int argc, char** argv) {
	ros::init(argc,argv,"quadcontrol");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("pidrequest", 3, control);
    pub = n.advertise<PID_control::motoroutputs>("pidreply", 3);
    ros::Rate freq(60);
    ROS_INFO("PID message control node running");
    while (ros::ok()) {
        #if throttled
            freq.sleep();
        #endif
        ros::spinOnce();
    }
    //Vertical control:
	return 0;
}