#include <Servo.h>
#include <IBusBM.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
/*

  (2)   (3)
      X
  (1)   (0)



*/




//----------------------------------------------------DEFENITIONS
//def for radio
#define PITCH 0
#define ROLL 1
#define THROTTLE 2
#define YAW 3
#define ARM 4
//def for setpoints
#define X 0
#define Y 1
#define Z 2
//def for gyro data

//def for pin
#define ESC_0 3
#define ESC_1 5
#define ESC_2 6
#define ESC_3 9
#define INTERRUPT_PIN 2

//----------------------------------------------------VARIABLES
int setpoint[3];  // 3d vector
int gyro_data[3];
int acc_data[3];
int channelValues [6];
int motorVal[4];

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw, pRoll, pPitch,pYaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

Servo m0;
Servo m1;
Servo m2;
Servo m3;

IBusBM IBus; // IBus object
MPU6050 mpu;

//=============================================================================================
//====================================== S E T U P ============================================
//=============================================================================================
void setup() {

  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  calculate_IMU_error();
  initMotors();
  //IBus.begin(Serial);
  //calibrateESC();


}

//=============================================================================================
//======================================== L O O P ============================================
//=============================================================================================

void loop() {

  //motor test----------------------------------------------------------
  /*
    motorSpeed(10,10,10,10);
    delay(2000);
    motorSpeed(500,500,500,500);
    delay(2000);
    motorSpeed(1000,1000,1000,1000);
    delay(2000);
    motorSpeed(1500,1500,1500,1500);
    delay(2000);
  */

  //gyro test------------------------------------------------------------

  //getChannelValues();
  readGyro();
  //processMotorSpeed();
  //testRadio();

}

//=============================================================================================
//=================================== F U N C T I O N S =======================================
//=============================================================================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ GENERIC ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void intiSerial()
{
  Serial.begin(9600);
}
void initI2C();

void prnt(String txt)
{
  Serial.print(txt);
}

void prntl(String txt)
{
  Serial.println(txt);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SENSOR ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void initGyro()
{
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
   // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
void readGyro()
{
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  // Print the values on the serial monitor
  /*Serial.print(roll);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.println(yaw);
  Serial.print("\t");*/
  Serial.print((roll-pRoll)*20+20);
  Serial.print("\t");
  Serial.print((pitch-pPitch)*20);
  Serial.print("\t");
  Serial.println((yaw-pYaw)*20-20);

  pRoll = roll;
  pPitch = pitch;
  pYaw = yaw;
  
}
void readAcc(int* data);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MOTOR ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void initMotors()
{
  /*
    pinMode(ESC_0, OUTPUT);
    pinMode(ESC_1, OUTPUT);
    pinMode(ESC_2, OUTPUT);
    pinMode(ESC_3, OUTPUT);
  */
  m0.attach(ESC_0, 1000, 2000);
  m1.attach(ESC_1, 1000, 2000);
  m2.attach(ESC_2, 1000, 2000);
  m3.attach(ESC_3, 1000, 2000);
  motorSpeed(0, 0, 0, 0);
  delay(2000);
}

void motorSpeed(int motor0, int motor1, int motor2, int motor3)
{
  //analogWrite(ESC_0, motor0);
  m0.writeMicroseconds(motor0 + 1000);
  m1.writeMicroseconds(motor1 + 1000);
  m2.writeMicroseconds(motor2 + 1000);
  m3.writeMicroseconds(motor3 + 1000);
  /*
    m0.write(motor0);
    m1.write(motor1);
    m2.write(motor2);
    m3.write(motor3);
  */
  /*
    analogWrite(ESC_1, motor1);
    analogWrite(ESC_2, motor2);
    analogWrite(ESC_3, motor3);
  */
}

void calibrateESC() // doesnt do anything
{
  motorSpeed(1000, 1000, 1000, 1000);
  Serial.println("ready to calibrate");
  while (Serial.read() != 'c')
  {
  }
  Serial.println("throttle down");
  motorSpeed(0, 0, 0, 0);
  delay(2000);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ RECIEVER ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void getChannelValues() //--------------------------------------------save channel values to an array
{
  for (int i = 0; i < 6; ++i)
  {
    channelValues[i] = IBus.readChannel(i) - 1500;
  }
  if (channelValues[1] > 0)digitalWrite(13, HIGH);
  else digitalWrite(13, LOW);
  return;
}

void testRadio()
{
  motorSpeed(channelValues[0], channelValues[1], channelValues[2], channelValues[3]);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PROCESSING ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void processMotorSpeed()
{
  motorVal[0] = 500 + channelValues[THROTTLE] - channelValues[ROLL] + channelValues[PITCH];
  motorVal[1] = 500 + channelValues[THROTTLE] + channelValues[ROLL] + channelValues[PITCH];
  motorVal[2] = 500 + channelValues[THROTTLE] + channelValues[ROLL] - channelValues[PITCH];
  motorVal[3] = 500 + channelValues[THROTTLE] - channelValues[ROLL] - channelValues[PITCH];
  motorSpeed(motorVal[0], motorVal[1], motorVal[2], motorVal[3]);
}
