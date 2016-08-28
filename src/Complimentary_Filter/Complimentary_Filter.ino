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

float dt, pitchAngle, rollAngle, Linear_Accel_Sensitivity, Angular_Rate_Sensitivity, Gyro_X_Offset, Gyro_Y_Offset, Gyro_Z_Offset;
float Gyro_X_Total, Gyro_Y_Total, Gyro_Z_Total;

int loopCounter;
void setup()
{
  Linear_Accel_Sensitivity = getLinearAccellerationSensitivity(Linear_Accel_Full_Scale);
  Angular_Rate_Sensitivity = getAngularRateSensitivity(Angular_Rate_Full_Scale);
  //The gyro has minor imperfections so we have to calibrate it in order to zero it out
  Gyro_X_Offset = 2.1756;
  Gyro_Y_Offset = -11.1621;
  Gyro_Z_Offset = -3.3401;
  dt = 0.01;//micros();
  loopCounter = 0;
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
  imu.read();  
  //UpdateGyroOffSets((imu.g.x*Angular_Rate_Sensitivity), (imu.g.y*Angular_Rate_Sensitivity), (imu.g.z*Angular_Rate_Sensitivity));
  dt = (millis() - dt)/1000;
  binaryFloat b_pitch, b_roll, b_ax, b_ay, b_az, b_gx, b_gy, b_gz;

  b_ax.floatingPoint = (imu.a.x*Linear_Accel_Sensitivity);
  b_ay.floatingPoint = (imu.a.y*Linear_Accel_Sensitivity);
  b_az.floatingPoint = (imu.a.z*Linear_Accel_Sensitivity);

  float accelData[3] = {b_ax.floatingPoint, b_ay.floatingPoint, b_az.floatingPoint};

  b_gx.floatingPoint = (imu.g.x*Angular_Rate_Sensitivity) - Gyro_X_Offset;//*dt;
  b_gy.floatingPoint = (imu.g.y*Angular_Rate_Sensitivity) - Gyro_Y_Offset;//*dt;
  b_gz.floatingPoint = (imu.g.z*Angular_Rate_Sensitivity) - Gyro_Z_Offset;//*dt;

  float gyroData[3] = {b_gx.floatingPoint, b_gy.floatingPoint, b_gz.floatingPoint};

  ComplementaryFilter(accelData, gyroData);

  b_pitch.floatingPoint = pitchAngle;
  b_roll.floatingPoint = rollAngle;
  
  Serial.write("S");
  Serial.write(b_pitch.binary, 4);
  Serial.write(b_roll.binary, 4);  
  //Serial.write(b_ax.binary, 4);
  //Serial.write(b_ay.binary, 4);
  Serial.write(b_az.binary, 4);
  Serial.write(b_gx.binary, 4);
  Serial.write(b_gy.binary, 4);
  Serial.write(b_gz.binary, 4); 
  delay(100);
}

void ComplementaryFilter(float accelData[3], float gyroData[3]){  

  float pA = pitchAngle + gyroData[0] * dt; //x angle
  float rA = rollAngle - gyroData[1] * dt;  //y angle

  int forceMagnitudeApprox = abs(accelData[0]) + abs(accelData[1]) + abs(accelData[2]);
  //if (forceMagnitudeApprox > 0.5 && forceMagnitudeApprox < 2){
    float pitchAccel = atan2(accelData[1], accelData[2]) * 180/M_PI;
    pitchAngle = (pA * 0.95) + (pitchAccel * 0.05);
    
    float rollAccel = atan2(accelData[0], accelData[2]) *180/M_PI;
    rollAngle = (rA * 0.95) + (rollAccel * 0.05);
  //}
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
 */
float getAngularRateSensitivity(short fullScaleValue){
  float rate = 0.0;
  switch(fullScaleValue){
    case 125:
      rate = 0.004375;
    break;
    case 245:
      rate = 0.00875;
    break;
    case 500:
      rate = 0.01750;
    break;
    case 1000:
      rate = 0.035;
    break;
    case 2000:
      rate = 0.070;
    break;
  }
  return rate;
}

void UpdateGyroOffSets(float x, float y, float z){
  loopCounter += 1;
  Gyro_X_Total += x;
  Gyro_Y_Total += y;
  Gyro_Z_Total += z;

  Gyro_X_Offset = Gyro_X_Total/loopCounter;
  Gyro_Y_Offset = Gyro_Y_Total/loopCounter;
  Gyro_Z_Offset = Gyro_Z_Total/loopCounter;
}

