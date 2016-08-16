#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <rosserial_arduino/Adc.h>
#include <sstream>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const rosserial_arduino::Adc::ConstPtr& msg)
{
  ROS_INFO("I heard the pinns: (%d, %d, %d, %d, %d, %d)", msg->adc0, msg->adc1, msg->adc2, msg->adc3, msg->adc4, msg->adc5);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber subscriber_imu = n.subscribe("adc", 1000, chatterCallback);
  ros::spin();
  return 0;
}
