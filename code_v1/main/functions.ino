void intiSerial()
{
  Serial.begin(9600);
}
void initI2C();
void readGyro(int* data);
void readAcc(int* data);

void initMotors()
{
  /*
  pinMode(ESC_0, OUTPUT);
  pinMode(ESC_1, OUTPUT);
  pinMode(ESC_2, OUTPUT);
  pinMode(ESC_3, OUTPUT);
  */
  m0.attach(ESC_0,1000,2000);
  m1.attach(ESC_1,1000,2000);
  m2.attach(ESC_2,1000,2000);
  m3.attach(ESC_3,1000,2000);
}

void motorSpeed(int motor0, int motor1, int motor2,int motor3)
{
  //analogWrite(ESC_0, motor0);
  m0.writeMicroseconds(motor0+1000);
  m1.writeMicroseconds(motor1+1000);
  m2.writeMicroseconds(motor2+1000);
  m3.writeMicroseconds(motor3+1000);
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
  motorSpeed(1000,1000,1000,1000);
  Serial.println("ready to calibrate");
  while(Serial.read()!='c')
  {
  }
  Serial.println("throttle down"); 
  motorSpeed(0,0,0,0);
  delay(2000);
}
