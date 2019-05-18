#include "ros/ros.h"
#include "reader/floatStamped.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

using namespace message_filters;


void differential_drive_kinematics(const reader::floatStamped::ConstPtr& leftSpeed, const reader::floatStamped::ConstPtr& rightSpeed,
        const reader::floatStamped::ConstPtr& steer)
{
    //kinematics algorithm
}

void ackerman_model(const reader::floatStamped::ConstPtr& leftSpeed, const reader::floatStamped::ConstPtr& rightSpeed,
        const reader::floatStamped::ConstPtr& steer)
{
    //ackerman algorithm
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "read");
    ros::NodeHandle n;
    
    Subscriber<reader::floatStamped> read_L(n, "/speedL_stamped", 1000);
    Subscriber<reader::floatStamped> read_R(n, "/speedR_stamped", 1000);
    Subscriber<reader::floatStamped> steer(n, "/steer_stamped", 1000);
    
    typedef sync_policies::ApproximateTime<reader::floatStamped, reader::floatStamped, reader::floatStamped> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), read_L, read_R, steer);
    
    switch(/*TODO condition*/) {
        case /*case1*/:
            sync.registerCallback(boost::bind(&differential_drive_kinematics, _1, _2, _3));
            break;
        case /*case2*/:
            sync.registerCallback(boost::bind(&ackerman_model, _1, _2, _3));
            break;
    }
    
    ros::spin();

    return 0;
}
