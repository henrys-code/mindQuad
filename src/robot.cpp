#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

using namespace ros;

void imu_handler(const geometry_msgs::Twist::ConstPtr& imu_data)
{
}

int main(int argc, char **argv)
{
    init(argc, argv, "robot");
    NodeHandle n;
    Rate r(10);
    Subscriber imu_sub = n.subscribe("/imu_data", 1000, imu_handler);

    return 0;
}
