/*
 *    Author: Henry Gaudet
 *    A flight controller for the Arduino Uno that accepts CPPM R/C input,
 *    Gyro, Accelerometer, Magnetometer, and Pressure sensor readings,
 *    calculates PID attitude control, and outputs a control signal to 4 
 *    ESCs that control electric motors.
 */

#include <Wire.h>
#include <LSM6.h>
#include <LPS.h>
#include <LIS3MDL.h>
#include <CPPM.h>

#define   MIN_VOLTAGE   1050

#define   P_GAIN_ROLL   1.0
#define   I_GAIN_ROLL   0.0
#define   D_GAIN_ROLL   0.0
#define   MAX_ROLL      400

#define   P_GAIN_PITCH  1.0
#define   I_GAIN_PITCH  0.0
#define   D_GAIN_PITCH  0.0
#define   MAX_PITCH     400

#define   P_GAIN_YAW    1.0
#define   I_GAIN_YAW    0.0
#define   D_GAIN_YAW    0.0
#define   MAX_YAW       400

LSM6 imu;
LPS altimeter;
LIS3MDL magnet;
LIS3MDL::vector<int16_t> running_min = {32767, 32767, 32767}, 
                         running_max = {-32768, -32768, -32768};
uint32_t user_pitch_in, user_roll_in, user_throttle_in, user_yaw_in;
uint32_t signal_out_1, signal_out_2, signal_out_3, signal_out_4;
uint32_t timer, dt;
uint16_t battery_voltage;
int esc_1, esc_2, esc_3, esc_4;
float gyro_roll, gyro_pitch, gyro_yaw;
float roll_off, pitch_off, yaw_off;
float pressure, altitude, temperature;
float roll_des, pitch_des, yaw_des;
float roll_err, pitch_err, yaw_err;
float prev_roll_err, prev_pitch_err, prev_yaw_err;
float roll_err_sum, pitch_err_sum, yaw_err_sum;
float roll_output, pitch_output, yaw_output;


void LED_on()
{
  PORTB |= 0x20;
}

void LED_off()
{
  PORTB &= 0xDF;
}

void BlinkLED(int milliseconds)
{

      LED_on();
      delay(milliseconds);
      LED_off();
      delay(milliseconds);
      QuietMotors();

}

// Reads IMU data into roll, pitch, yaw vars
void ReadIMU() 
{
  
  imu.read();
  
  // Complementary filter: 0.8*old + 0.2*new
  gyro_roll =  0.8*gyro_roll + 0.2*(imu.g.x - roll_off) / 57.1428571429;
  gyro_pitch = 0.8*gyro_pitch + -0.2*(imu.g.y - pitch_off) / 57.1428571429;
  gyro_yaw = 0.8*gyro_yaw + -0.2*(imu.g.z - yaw_off) / 57.1428571429;

}

// Calibrate the IMU to offset drift
void CalibrateIMU() 
{
  
  Serial.print("Calibrating IMU...");

  roll_off = pitch_off = yaw_off = 0.0;

  for (int i = 0; i < 2000; i ++) 
  {
    imu.read();
    roll_off += imu.g.x;
    pitch_off += imu.g.y;
    yaw_off += imu.g.z;
    
    if (i % 100 == 0) 
    {
      Serial.print(".");
    }
    QuietMotors(); // To stop annoying beeps
  }
  
  // divide to get average
  roll_off /= 2000;
  pitch_off /= 2000;
  yaw_off /= 2000;

  gyro_roll = (imu.g.x - roll_off) / 57.1428571429;
  gyro_pitch = (imu.g.y - pitch_off) / 57.1428571429;
  gyro_yaw = (imu.g.z - yaw_off) / 57.1428571429;
  
  Serial.println();

}

void ReadAltimeter()
{

  pressure = altimeter.readPressureMillibars();
  altitude = altimeter.pressureToAltitudeMeters(pressure);
  temperature = altimeter.readTemperatureC();

}

void CalibrateMagnet()
{

  Serial.print("Calibrating magnetometer...");
  for (int i=0; i < 2000; i++)
  {
    magnet.read();

    running_min.x = min(running_min.x, magnet.m.x);
    running_min.y = min(running_min.y, magnet.m.y);
    running_min.z = min(running_min.z, magnet.m.z);

    running_max.x = max(running_max.x, magnet.m.x);
    running_max.y = max(running_max.y, magnet.m.y);
    running_max.z = max(running_max.z, magnet.m.z);

    if (i % 100 == 0) {
      Serial.print(".");
    }
    QuietMotors();
  }
  
  Serial.print("\n");

}


void ReadMagnet()
{

  magnet.read();

}


void GetSensorData() {
    
  ReadIMU();
  ReadMagnet();
  ReadAltimeter();

}


void GetCtrlInputs() {
  
  CPPM.cycle();
  if (CPPM.synchronized())
  {
    user_pitch_in = CPPM.read_us(CPPM_ELEV);
    user_roll_in = CPPM.read_us(CPPM_AILE);
    user_throttle_in = CPPM.read_us(CPPM_THRO);
    user_yaw_in = CPPM.read_us(CPPM_RUDD);
  }

}


void QuietMotors()
{
  
    PORTD |= 0xF0;
    delayMicroseconds(1000);
    PORTD &= 0x0F;

}

void Hover(int altitude)
{
  BlinkLED(250);
}

void PID()
{

  // ROLL
  roll_err = gyro_roll - roll_des;
  roll_err_sum = roll_err_sum + roll_err * I_GAIN_ROLL;
  if (MAX_ROLL < roll_err_sum)  roll_err_sum = MAX_ROLL;
  else if (roll_err_sum < MAX_ROLL * -1)  roll_err_sum = MAX_ROLL * -1;

  roll_output = P_GAIN_ROLL * roll_err + roll_err_sum + D_GAIN_ROLL *(roll_err - prev_roll_err);

  if (MAX_ROLL < roll_output) roll_output = MAX_ROLL;
  else if (roll_output < MAX_ROLL * -1) roll_output = MAX_ROLL * -1;

  prev_roll_err = roll_err;

 
  // PITCH
  pitch_err = gyro_pitch - pitch_des;
  pitch_err_sum = pitch_err_sum + pitch_err * I_GAIN_PITCH;
  if (MAX_PITCH < pitch_err_sum)  pitch_err_sum = MAX_PITCH;
  else if (pitch_err_sum < MAX_PITCH * -1)  pitch_err_sum = MAX_PITCH * -1;

  pitch_output = P_GAIN_PITCH * pitch_err + pitch_err_sum + D_GAIN_PITCH *(pitch_err - prev_pitch_err);

  if (MAX_PITCH < pitch_output) pitch_output = MAX_PITCH;
  else if (pitch_output < MAX_PITCH * -1) pitch_output = MAX_PITCH * -1;

  prev_pitch_err = pitch_err;

  
  // YAW
  yaw_err = gyro_yaw - yaw_des;
  yaw_err_sum = yaw_err_sum + yaw_err * I_GAIN_YAW;
  if (MAX_YAW < yaw_err_sum)  yaw_err_sum = MAX_YAW;
  else if (yaw_err_sum < MAX_YAW * -1)  yaw_err_sum = MAX_YAW * -1;

  yaw_output = P_GAIN_YAW * yaw_err + yaw_err_sum + D_GAIN_YAW *(yaw_err - prev_yaw_err);

  if (MAX_YAW < yaw_output) yaw_output = MAX_YAW;
  else if (yaw_output < MAX_YAW * -1) yaw_output = MAX_YAW * -1;

  prev_yaw_err = yaw_err;
}


void setup()
{

  TWBR = 12;
  DDRB |= 0x30; // Set pins 12 & 13 as output
  DDRD |= 0xF0; // Set pins 4, 5, 6, & 7 as output (motors)
  
  battery_voltage = (analogRead(0) + 75) * 1.93858267717;
  prev_roll_err = prev_pitch_err = prev_yaw_err = 0.0;
  roll_des = pitch_des = yaw_des = 0.0;
  
  Serial.begin(9600);
  Serial.println("Begin setup");  
  Wire.begin();

  QuietMotors();
  // Start IMU
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    // Blink LED to signal sensor failed
    while (1)
    {
      BlinkLED(250);
    }
  }
  QuietMotors();
  imu.enableDefault();
  CalibrateIMU();


  // Start magnetometer
  if (!magnet.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
    // Blink LED to signal sensor failed
    while (1)
    {
      BlinkLED(750);
    }
  }
  magnet.enableDefault();
  CalibrateMagnet();


  // Start pressure/temperature sensor
  if (!altimeter.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    // Blink LED to signal sensor failed
    while (1)
    {
      BlinkLED(1500);
    }
  }

  altimeter.enableDefault();

  
  // Start R/C input receiver
  CPPM.begin();

  /*
   *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   * ! ! ! Throttle and yaw must be in lowest position to                   !!
   *       escape this loop and begin flight control loop ! ! !            !!
   *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   */ 
  int wait = 0;
  while (user_throttle_in < 990 || 1020 < user_throttle_in || 1020 < user_yaw_in)
  {
    
    QuietMotors();    // To stop annoying beeps

    // Get R/C inputs
    CPPM.cycle();
    if (CPPM.synchronized())
    {
      user_throttle_in = CPPM.read_us(CPPM_THRO);
      user_yaw_in = CPPM.read_us(CPPM_RUDD);
    }
    
    if (wait == 250)
    {
      // Blink LED to signal receiver is in wrong position
      digitalWrite(13, !digitalRead(13));
      Serial.println("THROTTLE STICK BOTTOM LEFT");
      wait = 0;
    }
    wait++;    
  }

  digitalWrite(13,LOW);
  Serial.println("Finished setup");
  timer = micros();

}



// Main flight control loop
void loop() 
{
  GetSensorData();
  GetCtrlInputs();

  battery_voltage = map(analogRead(0), 473, 580, 1036, 1260);
  //Serial.println(battery_voltage);
  if (battery_voltage < MIN_VOLTAGE && 600 < battery_voltage)
  {
    LED_on();
    // land() maybe?
  }
  
  // PID
  roll_des = pitch_des = yaw_des = 0.0;
  
  if (1508 < user_roll_in)      roll_des = (int)user_roll_in - 1508;
  else if (user_roll_in < 1492) roll_des = (int)user_roll_in - 1492;
  roll_des /= 3.0;

  if (1508 < user_pitch_in)      pitch_des = (int)user_pitch_in - 1508;
  else if (user_pitch_in < 1492) pitch_des = (int) user_pitch_in - 1492;
  pitch_des /= 3.0;

  if (1508 < user_yaw_in) yaw_des = (int)user_yaw_in - 1508;
  else if (user_yaw_in < 1492) yaw_des = (int)user_yaw_in - 1492;
  yaw_des /= 3.0;


  PID();
  
  if (1800 < user_throttle_in) user_throttle_in = 1800;

  esc_1 = user_throttle_in + pitch_output - roll_output - (yaw_output);  // Front Left CCW
  esc_2 = user_throttle_in + pitch_output + roll_output + (yaw_output);  // Front Right CW
  esc_3 = user_throttle_in - pitch_output + roll_output - (yaw_output);  // Rear Right CCW
  esc_4 = user_throttle_in - pitch_output - roll_output + (yaw_output);  // Rear Left CW

/*
  Serial.print(esc_1);
  Serial.print("\t");
  Serial.print(esc_2);
  Serial.print("\t");
  Serial.print(esc_3);
  Serial.print("\t");
  Serial.print(esc_4);
  Serial.print("|\t");
  Serial.print(gyro_yaw);
  Serial.print("\t");
  Serial.print(yaw_des);
  Serial.print("\t");
  Serial.print(yaw_output);
  Serial.print("|\t");
  Serial.print(battery_voltage);
  Serial.println();
*/

  if (800 < battery_voltage && battery_voltage < 1260)
  {
    esc_1 += esc_1 * ((1260 - battery_voltage)/3500.0);
    esc_2 += esc_2 * ((1260 - battery_voltage)/3500.0);
    esc_3 += esc_3 * ((1260 - battery_voltage)/3500.0);
    esc_4 += esc_4 * ((1260 - battery_voltage)/3500.0);
  }

  if (esc_1 < 1050) esc_1 = 1050;
  if (esc_2 < 1050) esc_2 = 1050;
  if (esc_3 < 1050) esc_3 = 1050;
  if (esc_4 < 1050) esc_4 = 1050;

  if (2200 < esc_1) esc_1 = 2200;
  if (2200 < esc_2) esc_2 = 2200;
  if (2200 < esc_3) esc_3 = 2200;
  if (2200 < esc_4) esc_4 = 2200;

  // Send signal to ESCs to power motors
  while(micros() < timer + 4000); // to keep signal synced
  timer = micros();
  PORTD |= 0xF0;
  signal_out_1 = timer + esc_1;
  signal_out_2 = timer + esc_2;
  signal_out_3 = timer + esc_3;
  signal_out_4 = timer + esc_4;
  
  while(16 <= PORTD)
  {
    uint32_t timer2 = micros();
    if(signal_out_1 <= timer2) PORTD &= 0xEF;
    if(signal_out_2 <= timer2) PORTD &= 0xDF;
    if(signal_out_3 <= timer2) PORTD &= 0xBF;
    if(signal_out_4 <= timer2) PORTD &= 0x7F;
  }
  
}
