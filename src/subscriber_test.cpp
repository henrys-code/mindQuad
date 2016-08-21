#include <ros/ros.h>
#include <std_msgs/String.h>


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    unsigned short ax;
    unsigned char *chars = reinterpret_cast<unsigned char *>(msg->data);
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber_test");
  ros::NodeHandle n;
  ros::Subscriber subscriber_imu = n.subscribe("o", 1000, chatterCallback);
  ros::spin();
  return 0;
}
