#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <rosserial_arduino/Adc.h>
#include <sstream>

void chatterCallback(const rosserial_arduino::Adc::ConstPtr& msg)
{
    ROS_INFO("Message 'o' received with data: %d", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber subscriber_imu = n.subscribe("o", 1000, chatterCallback);
  ros::spin();
  return 0;
}
