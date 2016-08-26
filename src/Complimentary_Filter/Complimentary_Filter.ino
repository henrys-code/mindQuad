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

#define M_PI 3.14159265359

typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;

LSM6 imu;
short Linear_Accel_Full_Scale = 2;
short Angular_Rate_Full_Scale = 245;

float dt, pitch, roll, Linear_Accel_Sensitivity, Angular_Rate_Sensitivity;

void setup()
{
  Linear_Accel_Sensitivity = getLinearAccellerationSensitivity(Linear_Accel_Full_Scale);
  Angular_Rate_Sensitivity = getAngularRateSensitivity(Angular_Rate_Full_Scale);
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
}

void loop()
{
  dt = (millis() - dt)/1000.0;
  imu.read();
  unsigned short accelData[3] = {imu.a.x, imu.a.y, imu.a.z};
  unsigned short gyroData[3] = {imu.g.x, imu.g.y, imu.g.z};

  //ComplementaryFilter(accelData, gyroData, pitch, roll, dt);

  binaryFloat b_ax, b_ay, b_az, b_gx, b_gy, b_gz;

  b_ax.floatingPoint = (imu.a.x*Linear_Accel_Sensitivity);
  b_ay.floatingPoint = (imu.a.y*Linear_Accel_Sensitivity);
  b_az.floatingPoint = (imu.a.z*Linear_Accel_Sensitivity);

  b_gx.floatingPoint = ((imu.g.x/Angular_Rate_Sensitivity)*dt);
  b_gy.floatingPoint = ((imu.g.y/Angular_Rate_Sensitivity)*dt);
  b_gz.floatingPoint = ((imu.g.z/Angular_Rate_Sensitivity)*dt);
  
  
  //Serial.write(b_pitch.binary, 4);
  //Serial.write(b_roll.binary, 4);  
  Serial.write(b_ax.binary, 4);
  Serial.write(b_ay.binary, 4);
  Serial.write(b_az.binary, 4);
  Serial.write(b_gx.binary, 4);
  Serial.write(b_gy.binary, 4);
  Serial.write(b_gz.binary, 4);
  delay(1000);
}

void ComplementaryFilter(unsigned short accelData[3], unsigned short gyroData[3], float& pitch, float& roll, float& dt){
  float pitchAccel, rollAccel;

  pitch += ((float)gyroData[0] / Angular_Rate_Sensitivity) * dt; // X Angle
  roll -= ((float)gyroData[1] / Angular_Rate_Sensitivity) * dt; // Y Angle

  int forceMagnitudeApprox = abs(accelData[0] + accelData[1] + accelData[2]);
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768){
    pitchAccel = atan2((float)accelData[1], (float)accelData[2]) * 180 / M_PI;
    pitch = pitch * 0.98 + pitchAccel * 0.02;
    
    rollAccel = atan2((float)accelData[0], (float)accelData[2]) * 180 / M_PI;
    roll = roll * 0.98 + rollAccel * 0.02;
  }
}

/*
 *returns a force in units of mg/LSB (milli g's / least significant bit) 
 *Default full-scale selection is +-2g
 *A value of 16,384 * (0.000061) corresponds to a 1g force
 */
float getLinearAccellerationSensitivity(short fullScaleValue){
  float force = 0.0;
  switch(fullScaleValue){
    case 2:
      force = 0.000061;
    break;
    case 4:
      force = 0.000122;
    break;
    case 8:
      force = 0.000244;
    break;
    case 16:
      force = 0.000488;
    break;
  }
  return force;
}

/*
 * The rate is in units of mdps/LSB (milli degrees per second / least significant bit)
 * Default full-scale selection is +-245 dps (for a range of 490 degrees)
 * A value of 16,384 * (0.00875) corresponds to a 0.11 degree angle per second
 */
float getAngularRateSensitivity(short fullScaleValue){
  float rate = 0.0;
  switch(fullScaleValue){
    case 125:
      rate = 4.375;
    break;
    case 245:
      rate = 8.75;
    break;
    case 500:
      rate = 17.50;
    break;
    case 1000:
      rate = 35;
    break;
    case 2000:
      rate = 70;
    break;
  }
  return rate;
}
