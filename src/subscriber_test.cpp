#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <sstream>

void chatterCallback(const std_msgs::Int16::ConstPtr& msg)
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
