
#include <Wire.h>
#include <Adafruit_L3GD20.h>
#define tachSlave 1

int rpm = 0;
int result_rpm;
int left_rpm;
boolean printcheck = true;
byte buff = 0;
int val = 0;
int rpmIn = 0;
int time = 0;
int elap = 0;
int timeold = 0;

int motorPin = 11; // Analog output pin that the motor is attached to
int throttle_arm = 94;
int throttle_start = 107;
int throttle_set = 107;
float throttle_ramp = 125;

double rpmReading = 0;
double rpmLastRead = 0;
double rpmSet = 0;
double driveControl = 0;
double driveControl_old = 0;
Adafruit_L3GD20 gyro;
double kp = 2;
//int ki = 2;
double kd = 2;
double pwmLimit = 120;
double pwmMax = 125;

int led = 13;
int potPin = 0;

//int valueInput;

void setup() {
  Serial.begin(115200);
  gyro.begin(gyro.L3DS20_RANGE_250DPS); //initialize gyro
  Wire.begin();
  pinMode(13,OUTPUT);
  digitalWrite(SDA,LOW);      // turn off internal I2C pull-up
  digitalWrite(SCL,LOW);      // turn off internal I2C pull-up
  
  Serial.println("check1");        // send first startup check signal to pi
  while(Serial.available()==0) {   // Wait for pi to find arduino serial port and respond
  }
  Serial.read();                        // clear serial port
  digitalWrite(led,!digitalRead(led));  // turn on led
  Serial.println("check2");             // send second startup check signal to pi

  while(Serial.available()==0) {        // pi startup check hands off to cam_new
  }                                     // wait for cam_new to respond
  int FrameSize = Serial.parseInt();      // Cam_new specifies camera frame size
  if (Serial.available()>0) {                // to speed up parseint, a terminator is added (100;)
    Serial.read();                           // Read terminator
  }
  digitalWrite(led,!digitalRead(led));       // turn off led, startup completed
  FrameSize = FrameSize;
  
  //-------- Initialize and arm Motor -------//
  //delay(10000);
  analogWrite(motorPin, 94); // initialize ESC at 0 Throttle
  //Serial.println("arm");
  delay(1000); 
  //analogWrite(motorPin,107);
  //Serial.println("start");
  //delay(9000);
  
  
  // Wait for input for motor drive
  while(Serial.available() == 0) {}
  val = Serial.parseInt();
  digitalWrite(led,!digitalRead(led));
  if(Serial.available() > 0) {
    buff = Serial.read();
    digitalWrite(led,!digitalRead(led));
  } 
}

void loop() {
    
//  if (val == 0) {
//    analogWrite(motorPin,94);
//    Serial.println("Stopped");
//  }
//  else {
    rpmSet = val;
    
    //rpm = 18000;
    rpm = getTach();
    rpmReading = rpm;
  
    
    double error = (rpmSet - rpmReading)/26500;
    double P = kp*error;
    double D = kd*((rpmReading - rpmLastRead)/26500);
    
    double scaler = (pwmMax-107)*((rpmSet-5600)/26500);
    double Pdrive = scaler*P;
    double Ddrive = scaler*D;
    driveControl = 107 + scaler + Pdrive + Ddrive;
    
    if (driveControl > pwmLimit) {
      driveControl = pwmLimit;
    }
    else if (driveControl < 107) {
      driveControl = 107;
    }
    analogWrite(motorPin,driveControl);
   // Serial.println(driveControl);
    
    rpmLastRead = rpmReading;
    driveControl_old = driveControl; 
    
    gyro.read();
    time = micros();
    elap = time - timeold;
    timeold = time;
    Serial.print(rpmSet);
    Serial.print(", ");
    Serial.print(rpmReading);
    Serial.print(", ");
    Serial.print(gyro.data.z);
    Serial.print(", ");
    Serial.print(elap);
    Serial.print(", ");
    Serial.println(driveControl_old);
  }
}

/*====== Request Tachometer ======*/  
int getTach() {
  Wire.requestFrom(tachSlave,1);   // Request byte of data
  while (Wire.available()) {
    result_rpm = Wire.read();      // Receive LSB right 8 bit portion
  }
  Wire.requestFrom(tachSlave,1);   // Request byte of data
  while (Wire.available()) {
    left_rpm = Wire.read();        // Receive MSB left 8 bit portion
    result_rpm |= (left_rpm << 8); // shift left portion 8-bits left, combine
  }
  return result_rpm;             
}
