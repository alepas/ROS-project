#include "ros/ros.h"
#include "project/floatStamped.h"
#include "project/custom_message.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "dynamic_reconfigure/server.h"
#include "project/algorithm_paramConfig.h"
#include "math.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

using namespace message_filters;

ros::Publisher publisher;
ros::Publisher customPublisher;

bool globalIsDDK;
double base = 1.3;
int steering_factor = 18;
float dist = 1.765;

ros::Time tk(0.0);
double xk = 0.0;
double yk = 0.0;
double thetak = 0.0;

class Variables {
    public:
        double x;
        double y;
        double theta;
        ros::Time time;
        double v;
        double omega;
};

Variables pose_evaluation(double omega, double radius, double linVel, const project::floatStamped::ConstPtr& steer){
     //delta computation
    ros::Time tk1 = steer->header.stamp;
    double delta;

    //using only the nanoseconds as delta for the first data
    if (tk.sec == 0) {
        delta = tk1.nsec;
    } else {
        delta = (tk1 - tk).toNSec();
    }

    //conversion from nanoseconds to seconds
    delta /= 1000000000;

    //new pose computation using 2nd order Runge-Kutta integration
    xk = xk + linVel * delta * cos(thetak + omega * delta / 2);
    yk = yk + linVel * delta * sin(thetak + omega * delta / 2);
    thetak = thetak + omega * delta;
    tk = tk1;

    Variables vars;
    vars.x = xk;
    vars.y = yk;
    vars.theta = thetak;
    vars.time = tk;
    vars.v = linVel;
    vars.omega = omega;
    return vars;
}

Variables differential_drive_kinematics(const project::floatStamped::ConstPtr& leftSpeed, 
                                   const project::floatStamped::ConstPtr& rightSpeed, 
                                   const project::floatStamped::ConstPtr& steer){
    //constants computation
    double omega = (rightSpeed->data - leftSpeed->data) / base;
    double radius;
    if(rightSpeed->data - leftSpeed->data == 0.0){
        radius = 0.0;
    }else{
        radius = (base / 2) * (rightSpeed->data + leftSpeed->data) / (rightSpeed->data - leftSpeed->data);
    }
    double linVel = omega * radius;

    return pose_evaluation(omega, radius, linVel, steer);
   
}

Variables ackerman_model(const project::floatStamped::ConstPtr& leftSpeed, 
                    const project::floatStamped::ConstPtr& rightSpeed,
                    const project::floatStamped::ConstPtr& steer){
    double alpha = steer->data / steering_factor;
    double radius = dist / tan(alpha);
    double rearSpeed = (leftSpeed->data + rightSpeed->data) / 2;

    double omega = rearSpeed * 1 / radius;
    double frontSpeed = omega * dist / sin(alpha);

    return pose_evaluation(omega, radius, frontSpeed, steer);

}

/* Dynamic_reconfigure callback, it assigns to a global variable the modified value of 
   isDDK param*/
void reconfigure(project::algorithm_paramConfig &config, uint32_t level){
    globalIsDDK = config.isDDK;
    xk = config.x;
    yk = config.y;
}

void publishing_func(Variables vars) {
    static tf::TransformBroadcaster broadcaster;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(vars.x, vars.y, 0));
    tf::Quaternion q;
    q.setRPY(0, 0, vars.theta);
    transform.setRotation(q);

    //publishing tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(vars.theta);
    broadcaster.sendTransform(tf::StampedTransform(transform, vars.time, "odom", "base_link"));

    //publishing on odom topic
    nav_msgs::Odometry odom;
    odom.header.stamp = vars.time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = vars.x;
    odom.pose.pose.position.y = vars.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vars.v * cos(vars.theta);
    odom.twist.twist.linear.y = vars.v * sin(vars.theta);
    odom.twist.twist.angular.z = vars.omega;
    
    publisher.publish(odom);
   
    project::custom_message msg;
    msg.stamp = vars.time;
    msg.frameId = "odom";
    msg.x = vars.x;
    msg.y = vars.y;
    msg.z = 0.0;
    msg.xTwist = odom_quat.x;
    msg.yTwist = odom_quat.y;
    msg.zTwist = odom_quat.z;
    msg.wTwist = odom_quat.w;
    msg.childFrameID = "base_link";
    msg.xLin = odom.twist.twist.linear.x;
    msg.yLin = odom.twist.twist.linear.y;
    msg.angLin = vars.omega;

    if(globalIsDDK){
        msg.type = "Differential dirve kinematics";
    }else{
        msg.type = "Ackerman Model";
    }

    customPublisher.publish(msg);
}

/* Sync callback: it the global var is set to True the DDK will be evaluate; if it is 
   set to False odometry will be done via Ackerman Mode*/  
void compute_algorithm(const project::floatStamped::ConstPtr& leftSpeed, 
                       const project::floatStamped::ConstPtr& rightSpeed,
                       const project::floatStamped::ConstPtr& steer) {
    Variables vars;
    if(globalIsDDK) {
	    vars = differential_drive_kinematics(leftSpeed, rightSpeed, steer);
    } else {
        vars = ackerman_model(leftSpeed, rightSpeed, steer);
    }

    publishing_func(vars);

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "reader");

    ros::NodeHandle r;
    ros::NodeHandle q;
    publisher = r.advertise<nav_msgs::Odometry>("odom", 1000);
    customPublisher = q.advertise<project::custom_message>("custom_message", 1000);

    ros::NodeHandle n;
    

    dynamic_reconfigure::Server<project::algorithm_paramConfig> server;
    dynamic_reconfigure::Server<project::algorithm_paramConfig>::CallbackType f;
    f = boost::bind(&reconfigure, _1, _2);
    server.setCallback(f);


    Subscriber<project::floatStamped> read_L(n, "/speedL_stamped", 1000);
    Subscriber<project::floatStamped> read_R(n, "/speedR_stamped", 1000);
    Subscriber<project::floatStamped> steer(n, "/steer_stamped", 1000);
    
    typedef sync_policies::ApproximateTime<project::floatStamped, project::floatStamped, project::floatStamped> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), read_L, read_R, steer);
         
    sync.registerCallback(boost::bind(&compute_algorithm, _1, _2, _3));

    ros::spin();

    return 0;
}