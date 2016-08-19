#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <sstream>

void chatterCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  ROS_INFO("I heard, Pose Position: (%f, %f, %f)", msg->position.x, msg->position.y, msg->position.z);
  ROS_INFO("I heard, Pose Orientation: (%f, %f, %f, %f)", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	std::cout << "x: " << msg->position.x << " y: " << msg->position.y << " z: " << msg->position.z << "\n";
	std::cout << "x: " << msg->orientation.x << " y: " << msg->orientation.y << " z: " << msg->orientation.z << " w: " << msg->orientation.w << "\n";

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub_imu_node");
  ros::NodeHandle n;
  ros::Subscriber subscriber_imu = n.subscribe("topic_imu", 1000, chatterCallback);
  ros::spin();
  return 0;
}
