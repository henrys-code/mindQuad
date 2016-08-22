/*
Example: An LSM6DS33 gives an accelerometer Z axis reading
of 16276 with its default full scale setting of +/- 2 g. The
LA_So specification in the LSM6DS33 datasheet (page 11)
states a conversion factor of 0.061 mg/LSB (least
significant bit) at this FS setting, so the raw reading of
16276 corresponds to 16276 * 0.061 = 992.8 mg = 0.9928 g.
*/

#include <avr/pgmspace.h>
#include <Wire.h>
#include <LSM6.h>
#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
std_msgs::Int16 msg;
ros::Publisher imu_publisher("o", &msg);

void setup()
{
    nh.initNode();
    nh.advertise(imu_publisher);
}

void loop()
{ 
  
  LSM6 imu;
  msg.data = 47;
  imu_publisher.publish(&msg);

//  imu.init();
//  imu.enableDefault();
//  imu.read();
  
  /*
  imu_msg.position.x = imu.a.x * 0.000061;
  imu_msg.position.y = imu.a.y * 0.000061;
  imu_msg.position.z = imu.a.z * 0.000061;

  imu_msg.orientation.x = imu.g.x * 0.00875;
  imu_msg.orientation.y = imu.g.y * 0.00875;
  imu_msg.orientation.z = imu.g.z * 0.00875;
  imu_msg.orientation.w = 0;
  */
  nh.spinOnce();
}
