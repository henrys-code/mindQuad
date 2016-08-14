#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>

using namespace ros;

int main (int argc, char **argv)
{
    init(argc, argv, "imu");
    NodeHandle n;
    Rate r(4);
    Publisher imu_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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
        for (i=0; i<move_list.size(); i++)
        {
            marker.pose.position.x += move_list[i][0];
            marker.pose.position.y += move_list[i][1];
            marker.pose.position.z += move_list[i][2];
            imu_pub.publish(marker);
            r.sleep();
        }


    }
}
