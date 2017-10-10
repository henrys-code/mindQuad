/*
  The sensor outputs provided by the library are the raw
  16-bit values obtained by concatenating the 8-bit high and
  low accelerometer and gyro data registers. They can be
  converted to units of g and dps (degrees per second) using
  the conversion factors specified in the datasheet for your
  particular device and full scale setting (gain).

  Example: An LSM6DS33 gives an accelerometer Z axis reading
  of 16276 with its default full scale setting of +/- 2 g. The
  LA_So specification in the LSM6DS33 datasheet (page 11)
  states a conversion factor of 0.061 mg/LSB (least
  significant bit) at this FS setting, so the raw reading of
  16276 corresponds to 16276 * 0.061 = 992.8 mg = 0.9928 g.
*/

#include <Wire.h>
#include <LSM6.h>

LSM6 imu;

int test = 1;
char report[80];
char calib[32];
float accel_x_offset, accel_y_offset, accel_z_offset;
float gyro_x_offset, gyro_y_offset, gyro_z_offset;
float dt;
float roll_angle = 0.0;
uint32_t prev_t, curr_t;


void calibrate_gyro() {

  float xsum = 0;
  float ysum = 0;
  float zsum = 0;
  int calibration_const = 3000;

  imu.read();

  for (int i = 0; i < calibration_const; i++) {
    xsum += imu.g.x * 0.00875;
    ysum += imu.g.y * 0.00875;
    zsum += imu.g.z * 0.00875;
  }
  gyro_x_offset = xsum / calibration_const;
  gyro_y_offset = ysum / calibration_const;
  gyro_z_offset = zsum / calibration_const;


}

void calibrate_accel() {

  float xsum = 0;
  float ysum = 0;
  float zsum = 0;
  int calibration_const = 3000;

  imu.read();

  for (int i = 0; i < calibration_const; i++) {
    xsum += imu.a.x * 0.061;
    ysum += imu.a.y * 0.061;
    zsum += imu.a.z * 0.061;
  }
  accel_x_offset = xsum / calibration_const;
  accel_y_offset = ysum / calibration_const;
  accel_z_offset = zsum / calibration_const;

  snprintf(calib, sizeof(calib), "\n\n%7d, %7d, %7d\n\n", accel_x_offset, accel_y_offset, accel_z_offset);
  Serial.println(calib);

}


void setup()
{
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
  calibrate_gyro();
  //calibrate_accel();
  accel_x_offset = accel_y_offset = accel_z_offset = 0;
  curr_t = micros();
}

void loop()
{
  imu.read();

  float gx = (imu.g.x * 0.00875) - gyro_x_offset;
  float gy = (imu.g.y * 0.00875) - gyro_y_offset;
  float gz = (imu.g.z * 0.00875) - gyro_z_offset;

  float ax = (imu.a.x * -0.000061) - accel_x_offset;
  float ay = (imu.a.y * 0.000061) - accel_y_offset;
  float az = (imu.a.z * -0.000061) - accel_z_offset;

  prev_t = curr_t;
  curr_t = micros();
  dt = (curr_t - prev_t) * 0.0001;

  //Serial.println("--------------------------");
  if (test) {
    test = 0;
  }
  else {
    roll_angle += gx / dt;
  }
  
  /*
    Serial.println(roll_angle);
    Serial.println(gx);
    Serial.println();
  */


  snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d    Angle: %6d",
           (int)ax, (int)ay, (int)az,
           (int)gx, (int)gy, (int)gz,
           (int)roll_angle);
  Serial.println(report);

  delay(100);

}
