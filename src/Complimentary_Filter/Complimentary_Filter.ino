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

float dt, pitchAngle, rollAngle
,Linear_Accel_Sensitivity, Angular_Rate_Sensitivity
,Gyro_X_Offset, Gyro_Y_Offset, Gyro_Z_Offset
,Accel_X_Offset, Accel_Y_Offset, Accel_Z_Offset;

void setup()
{
  Linear_Accel_Sensitivity = getLinearAccellerationSensitivity(Linear_Accel_Full_Scale);
  Angular_Rate_Sensitivity = getAngularRateSensitivity(Angular_Rate_Full_Scale);
    
  dt = 0.01;  
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
  delay(500);
  //The gyro has minor imperfections so we have to calibrate it in order to zero it out (2000 sample size)
  UpdateGyroOffSets(2000);
}

void loop()
{  
  imu.read();    
  dt = (millis() - dt)/1000;
  binaryFloat b_pitch, b_roll, b_ax, b_ay, b_az, b_gx, b_gy, b_gz;

  b_ax.floatingPoint = (imu.a.x*Linear_Accel_Sensitivity);
  b_ay.floatingPoint = (imu.a.y*Linear_Accel_Sensitivity);
  b_az.floatingPoint = (imu.a.z*Linear_Accel_Sensitivity);

  float accelData[3] = {b_ax.floatingPoint, b_ay.floatingPoint, b_az.floatingPoint};

  b_gx.floatingPoint = handleNoise((imu.g.x-Gyro_X_Offset)*Angular_Rate_Sensitivity, -5.0, 5.0);
  b_gy.floatingPoint = handleNoise((imu.g.y-Gyro_Y_Offset)*Angular_Rate_Sensitivity, -5.0, 5.0);
  b_gz.floatingPoint = handleNoise((imu.g.z-Gyro_Z_Offset)*Angular_Rate_Sensitivity, -5.0, 5.0);

  float gyroData[3] = {b_gx.floatingPoint, b_gy.floatingPoint, b_gz.floatingPoint};

  ComplementaryFilter(accelData, gyroData);
    
  b_pitch.floatingPoint = pitchAngle;
  b_roll.floatingPoint = rollAngle;

  binaryFloat s;
  s.floatingPoint = 0.0;
  
  Serial.write("S");
  Serial.write(b_pitch.binary, 4);
  Serial.write(b_roll.binary, 4);  
  //Serial.write(b_ax.binary, 4);
  //Serial.write(b_ay.binary, 4);
  Serial.write(s.binary, 4);
  Serial.write(b_gx.binary, 4);
  Serial.write(b_gy.binary, 4);
  Serial.write(b_gz.binary, 4); 
  delay(100);
}

void ComplementaryFilter(float accelData[3], float gyroData[3]){  

  pitchAngle = pitchAngle + gyroData[0] * dt; //x angle
  rollAngle = rollAngle - gyroData[1] * dt;  //y angle

  float forceMagnitudeApprox = abs(accelData[0]) + abs(accelData[1]) + abs(accelData[2]);
  //if (forceMagnitudeApprox > 0.5 && forceMagnitudeApprox < 2){
    float pitchAccel = atan2(accelData[1], accelData[2]) * 180/M_PI;
    pitchAngle = ((int)pitchAngle * 0.90) + (pitchAccel * 0.10);
    
    float rollAccel = atan2(accelData[0], accelData[2]) *180/M_PI;
    rollAngle = ((int)rollAngle * 0.90) + (rollAccel * 0.10);
 // }
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

void UpdateGyroOffSets(int numIterations){

  bool calibrated = false;
  float ax, ay, az, gx, gy, gz, testGyroReadingX, testGyroReadingY, testGyroReadingZ;
  while (!calibrated){
    //variables to hold the totals for accel/gyro x,y,z     
    //for every iteration
    for (int i = 0; i < numIterations; i++){
      //read the imu
      imu.read();
      //sum the values read
      ax += (imu.a.x);
      ay += (imu.a.y);
      az += (imu.a.z); 
      gx += (imu.g.x);
      gy += (imu.g.y);
      gz += (imu.g.z);

      //average the totals so we have a decent offset
      Accel_X_Offset = (ax/numIterations)*-1;
      Accel_Y_Offset = (ay/numIterations)*-1;
      Accel_Z_Offset = (az/numIterations)*-1;
      Gyro_X_Offset = gx/numIterations;
      Gyro_Y_Offset = gy/numIterations;
      Gyro_Z_Offset = gz/numIterations;
       
      //Take sample readings and account for minor noise
      testGyroReadingX = handleNoise((imu.g.x-Gyro_X_Offset) * Angular_Rate_Sensitivity, -3.0, 3.0);
      testGyroReadingY = handleNoise((imu.g.y-Gyro_Y_Offset) * Angular_Rate_Sensitivity, -3.0, 3.0);
      testGyroReadingZ = handleNoise((imu.g.z-Gyro_Z_Offset) * Angular_Rate_Sensitivity, -3.0, 3.0);
      //if all the sampled values are 0 then we are calibrated.
      if (testGyroReadingX < 0.01 && testGyroReadingX > -0.01 
        && testGyroReadingY < 0.01 && testGyroReadingY > -0.01
        && testGyroReadingZ < 0.01 && testGyroReadingZ > -0.01){
        calibrated = true;
      }
    }

    if (!calibrated){
      ax = ay = az = gx = gy = gz = 0.0;
    }
    
    binaryFloat aax, aay, aaz, ggx, ggy, ggz;
    aax.floatingPoint = Gyro_X_Offset;
    aay.floatingPoint = Gyro_Y_Offset;
    aaz.floatingPoint = Gyro_Z_Offset;
    ggx.floatingPoint = testGyroReadingX;
    ggy.floatingPoint = testGyroReadingY;
    ggz.floatingPoint = testGyroReadingZ;
    Serial.write("S");
    Serial.write(aax.binary, 4);
    Serial.write(aay.binary, 4);    
    Serial.write(aaz.binary, 4);
    Serial.write(ggx.binary, 4);
    Serial.write(ggy.binary, 4);
    Serial.write(ggz.binary, 4); 
  }
}

float handleNoise(float input, float lower, float upper){
  if (input > lower && input < upper){
    return 0.0;
  }
  return input;
}

