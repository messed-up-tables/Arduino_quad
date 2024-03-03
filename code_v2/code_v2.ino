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

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

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

  Serial.begin(115200);
  Serial.println("running");
  initMotors();
  //IBus.begin(Serial);
  //calibrateESC();
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  initGyro();


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
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
*/
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}
void readGyro()
{
  //Serial.println("reading gyro............");
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the latest packet

    // Get roll, pitch, and yaw from the sensor:
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Print the roll, pitch, and yaw values to the serial monitor:
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI); // Yaw
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI); // Pitch
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI); // Roll
  }

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
