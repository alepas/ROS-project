#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"



#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"


using namespace message_filters;
using namespace sensor_msgs;
using namespace geometry_msgs;

/*
    + SENSOR_MSGS/IMU STRUCTURE --> http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
    + SENSOR_MSGS/NAVSATFIX --> http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
    + GEOMETRY_MSGS/POINTSTAMPED --> http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html
*/ 

void compute_algorithm(const PointStamped::ConstPtr& speedSteer,
                       const Imu::ConstPtr& imu,
                       const NavSatFix::ConstPtr& gps){
    
    printf("Odometry values\n");
    printf("Header sec --> [%i]\n", speedSteer->header.stamp.sec);
    printf("Header nsec --> [%i]\n\n", speedSteer->header.stamp.nsec);
    printf("x value--> [%f]\n", speedSteer->point.x);
    printf("y value--> [%f]\n\n\n", speedSteer->point.y);

    printf("Imu values\n");
    printf("Header sec --> [%i]\n", imu->header.stamp.sec);
    printf("Header nsec --> [%i]\n\n\n", imu->header.stamp.nsec);

    printf("GPS values\n");
    printf("Header sec --> [%i]\n", gps->header.stamp.sec);
    printf("Header nsec --> [%i]\n\n", gps->header.stamp.nsec);
    printf("Status --> [%i]\n", gps->status.status);
    printf("Latitude --> [%f]\n", gps->latitude);
    printf("Longitude --> [%f]\n", gps->longitude);
    printf("Altitude --> [%f]\n", gps->altitude);
    printf("---\n\n\n");

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "reader");
    ros::NodeHandle n;

    Subscriber<PointStamped> speedSteer(n, "/speedsteer", 1000);
    Subscriber<Imu> imu(n, "/swiftnav/rear/imu", 1000);
    Subscriber<NavSatFix> gps(n, "/swiftnav/rear/gps", 1000);
    
    typedef sync_policies::ApproximateTime< PointStamped, Imu, NavSatFix> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), speedSteer, imu, gps);
         
    sync.registerCallback(boost::bind(&compute_algorithm, _1, _2, _3));
    ros::spin();

    return 0;
}
