#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_imu");
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   * Loop rate of 10Hz
   */
  ros::Publisher publisher_imu = n.advertise<geometry_msgs::Pose>("topic_imu", 1000);
  ros::Rate loop_rate(10);

  geometry_msgs::Pose pose;
  geometry_msgs::Point point;
  geometry_msgs::Quaternion quat;
  geometry_msgs::Vector3 accel;
  geometry_msgs::Vector3 angular_velocity;
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
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

    pose.position = point;
    pose.orientation = quat;

    /*std::stringstream ss;
    ss << "hello world " << count;
    */

    ROS_INFO("Pose Postion: (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);
    ROS_INFO("Pose Orientation: (%f, %f, %f, %f)", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    publisher_imu.publish(pose);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
