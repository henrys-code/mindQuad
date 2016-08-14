#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

#include <sstream>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  ROS_INFO("I heard, Pose Position: (%f, %f, %f)", msg->position.x, msg->position.y, msg->position.z);
  ROS_INFO("I heard, Pose Orientation: (%f, %f, %f, %f)", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber subscriber_imu = n.subscribe("topic_imu", 1000, chatterCallback);
  ros::spin();
  return 0;
}
