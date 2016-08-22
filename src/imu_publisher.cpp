#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_imu");
  ros::NodeHandle n;

  ros::Publisher publisher_imu = n.advertise<geometry_msgs::Pose>("topic_imu", 1000);
  ros::Rate loop_rate(10);

  geometry_msgs::Pose pose;
  geometry_msgs::Point point;
  geometry_msgs::Quaternion quat;
  geometry_msgs::Vector3 accel;
  geometry_msgs::Vector3 angular_velocity;
  
  point.x = 0.0f;
  point.y = 0.0f;
  point.z = 0.0f;

  quat.x = 0.0f;
  quat.y = 1.0f;
  quat.z = 0.0f;
  quat.w = 0.0f;

  accel.x = 0.0f;
  accel.y = 0.0f;
  accel.z = 0.0f;

  angular_velocity.x = 0.0f;
  angular_velocity.y = 0.0f;
  angular_velocity.z = 0.0f;

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    point.z += 0.1f;
    if (point.z > 3.0f)  point.z = 0.0f;

    pose.position = point;
    pose.orientation = quat;

    //ROS_INFO("Pose Postion: (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);
    //ROS_INFO("Pose Orientation: (%f, %f, %f, %f)", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    publisher_imu.publish(pose);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
