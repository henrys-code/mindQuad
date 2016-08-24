/*
Example: An LSM6DS33 gives an accelerometer Z axis reading
of 16276 with its default full scale setting of +/- 2 g. The
LA_So specification in the LSM6DS33 datasheet (page 11)
states a conversion factor of 0.061 mg/LSB (least
significant bit) at this FS setting, so the raw reading of
16276 corresponds to 16276 * 0.061 = 992.8 mg = 0.9928 g.
*/
#include <avr/pgmspace.h>   // Allows mem storage on flash instead of SRAM
#include <Wire.h>
#include <LSM6.h>           // IMU library
#include <Servo.h>          // Interfaces with motors

LSM6 imu;

void setup()
{
  Serial.begin(9600);
  //imu.init();
  //imu.enableDefault();
}

void loop()
{ 
  //imu.read();
  uint16_t ax = 10;
  uint16_t ay = 24;
  uint16_t az = 32;
  uint16_t gx = 41;
  uint16_t gy = 50;
  uint16_t gz = 67;

  Serial.write(ax);
  Serial.write(ay);
  Serial.write(az);
  Serial.write(gx);
  Serial.write(gy);
  Serial.write(gz);
  
  delay(1000);
}
