#include "ros/ros.h"
#include "reader/floatStamped.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "dynamic_reconfigure/server.h"
#include "project/algorithm_paramConfig.h"

using namespace message_filters;
bool globalIsDDK;

void differential_drive_kinematics(const reader::floatStamped::ConstPtr& leftSpeed, 
                                   const reader::floatStamped::ConstPtr& rightSpeed, 
                                   const reader::floatStamped::ConstPtr& steer){
    //kinematics algorithm
}

void ackerman_model(const reader::floatStamped::ConstPtr& leftSpeed, 
                    const reader::floatStamped::ConstPtr& rightSpeed,
                    const reader::floatStamped::ConstPtr& steer){
    //ackerman algorithm
}

/* Dynamic_reconfigure callback, it assigns to a global variable the modified value of 
   isDDK param*/
void getIsDDK(project::algorithm_paramConfig &config, uint32_t level){
    globalIsDDK = config.isDDK;
}

/* Sync callback: it the global var is set to True the DDK will be evaluate; if it is 
   set to False odometry will be done via Ackerman Mode*/  
void compute_algorithm(const project::floatStamped::ConstPtr& leftSpeed, 
                       const project::floatStamped::ConstPtr& rightSpeed, c
                       onst project::floatStamped::ConstPtr& steer){
    if(globalIsDDK){
	differential_drive_kinematics(leftSpeed, rightSpeed, steer);
    }
    else{ackerman_model(leftSpeed, rightSpeed, steer);}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "reader");
    ros::NodeHandle n;
    

    dynamic_reconfigure::Server<project::algorithm_paramConfig> server;
    dynamic_reconfigure::Server<project::algorithm_paramConfig>::CallbackType f;
    f = boost::bind(&getIsDDK, _1, _2);
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
