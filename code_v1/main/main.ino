#include <Servo.h>

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


//----------------------------------------------------VARIABLES
int radio[6];     // the transmitter has 6 channels. so an array of 6 would be required to handle that in put
int setpoint[3];  // 3d vector 
int gyro_data[3];
int acc_data[3];

//----------------------------------------------------PROTOTYPES
void readRadio(int* radio);   //return radio[6]
void calSetPoint(int* setpoint); //return setpoint[3]
void initSerial();
void initI2C();
void readGyro(int* data);
void readAcc(int* data);
void initMotors();
void calibrateESC();
void motorSpeed(int motor0, int motor1, int motor2, int motor3);
Servo m0;
Servo m1;
Servo m2;
Servo m3;

void setup() {

  Serial.begin(9600);
  initMotors();
  //calibrateESC();
  
}


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

}
