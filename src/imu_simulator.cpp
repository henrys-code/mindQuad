#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <geometry_msgs/Vector3.h>

using namespace ros;
geometry_msgs::Vector3 current_position;
geometry_msgs::Vector3 angular_velocity;


void callback_imu(const geometry_msgs::Pose::ConstPtr& msg)
{
  ROS_INFO("I heard, Pose Position1: (%f, %f, %f)", msg->position.x, msg->position.y, msg->position.z);
  current_position.x = msg->position.x;
  current_position.y = msg->position.y;
  current_position.z = msg->position.z;
  ROS_INFO("I heard, Pose Orientation1: (%f, %f, %f, %f)", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  angular_velocity.x = msg->orientation.x;
  angular_velocity.y = msg->orientation.y;
  angular_velocity.z = msg->orientation.z;
}

int main (int argc, char **argv)
{
    init(argc, argv, "imu");
    NodeHandle n;
    Rate r(4);
    Publisher imu_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	Subscriber subscriber_imu = n.subscribe("topic_imu", 1000, callback_imu);	
    int i, j;
    uint32_t shape = visualization_msgs::Marker::ARROW;
    std::vector< std::vector<int> > move_list(10, std::vector<int>(3,0));
    std::vector<int> move(3);
    move.push_back(1);
    move.push_back(0);
    move.push_back(0);
    for (i=0; i<4; i++)
    {
        move_list.push_back(move);
    }
    move[0] = 0;
    move[1] = 1;
    move[2] = 0;
    for (i=0; i<3; i++)
    {
        move_list.push_back(move);
    }
    move[0] = -1;
    move[1] = 0;
    for (i=0; i<3; i++)
    {
        move_list.push_back(move);
    }
    
    while (ok())
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/imu_frame";
        marker.header.stamp = Time::now();
        marker.ns = "imu";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        while (imu_pub.getNumSubscribers() < 1)
        {
            if (!ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        imu_pub.publish(marker);

        marker.pose.position.x = current_position.x;
        marker.pose.position.y = current_position.y;
        marker.pose.position.z = current_position.z;
		imu_pub.publish(marker);
		spinOnce();
		ROS_INFO("I heard, Pose Position9: (%f, %f, %f)", current_position.x, current_position.y, current_position.z);
        r.sleep();		
/*
        for (i=0; i<move_list.size(); i++)
        {
            marker.pose.position.x += move_list[i][0];
            marker.pose.position.y += move_list[i][1];
            marker.pose.position.z += move_list[i][2];
            imu_pub.publish(marker);
            r.sleep();
        }
*/

    }
}
