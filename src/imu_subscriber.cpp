#include <ros/ros.h>
#include <std_msgs/String.h>

void data_callback(const std_msgs::String::ConstPtr& msg)
{
    std::string s = msg->data.c_str();
    short imu_values[6];
    int size = 12, i = 0;
    char imu_data[12];
    for (i = 0; i < 6; i++)
    {
        imu_values[i] = 0;
    }
    for (i = 0; i < size; i++)
    {
        imu_data[i] = s[i];
    }
    imu_values[0] = (imu_data[0]<<8)|imu_data[1];
    imu_values[1] = (imu_data[2]<<8)|imu_data[3];
    imu_values[2] = (imu_data[4]<<8)|imu_data[5];
    imu_values[3] = (imu_data[6]<<8)|imu_data[7];
    imu_values[4] = (imu_data[8]<<8)|imu_data[0];
    imu_values[5] = (imu_data[10]<<8)|imu_data[11];
    
    for (i = 0; i < 6; i++)
    {
        std::cout << imu_values[i] << std::endl;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub_imu_node");
  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe("o", 1000, data_callback);
  ros::spin();
  return 0;
}
