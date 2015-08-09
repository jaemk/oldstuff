/* Arduino code to control Motor Controller (Master - ArduinoMega) */
/* Reaction wheel spins clockwise */

#include <Wire.h>
#include <Adafruit_L3GD20.h>
#define tachSlave 1           // define I2C device locations
#define solSlave 3

Adafruit_L3GD20 gyro;
int motorPin = 11;
int led = 13;
int sol = 5;
int soldelay = 15;  // milliseconds

// I2C Transfer variables
int rpm = 0;
int result_rpm;
int left_rpm;
boolean printcheck = true;
byte buff = 0;

int PiCom = 0;
int flipdelay = 500;
boolean serialcheck = false;
boolean check = false;
boolean servocheck = false;
int timereading = 0;
int timeold = 0;

// Mass Moments of Inertia
float moi_frame = 0.03923;      //ALL MOI'S kg*m^2
float moi_fwa = 9.2212e-5;

// gyro filtering and calculation
float rpm2rads = 0.1047;
double gyro_thresh_top = 1.00;
double gyro_thresh_bot = -1.00;
double gyroReading = 0;
float gyro_calc = 0;
int index_gyro = 0;
double gyroStore[3] = {0,0,0};
double gyroSum = 0;
double gyroValue = 0;


// PID Drive Variables
double rpmReading = 0;
double rpmLastRead = 0;
double rpmSet = 0;
double val = 0;
double driveControl = 0;
double driveControl_old = 0;
double kp = 41;              // constant gains
double ki = 0;               // ""
double kd = 0;               // ""
double P = 0;
double I = 0;
double D = 0;
double scaler = 0;
double Pdrive = 0;
double Idrive = 0;
double Ddrive = 0;
double pwmLimit = 120;  // used to limit output, but not limit scale
double pwmMax = 125;
double pwmMin = 107;
double pwmNom = 109;
int pwmArm = 94;
double rpmMax = 26500;
double rpmMin = 5600;
double rpmNom = 10000;
double rpmThreshHigh = 18000;
double rpmThreshLow = 6000;

double error = 0;
double error_tot = 0;
int error_moving[10] = {0,0,0,0,0,0,0,0,0,0};
double error_moving_tot = 0;
int index = 0;
double error_moving_avg = 0;

double rpmDrive = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  gyro.begin(gyro.L3DS20_RANGE_250DPS);   // initialize gyro
  pinMode(led,OUTPUT);
  pinMode(sol,OUTPUT);
  digitalWrite(SDA,LOW);      // turn off internal I2C pull-up
  digitalWrite(SCL,LOW);      // turn off internal I2C pull-up

  /*------ Connect to Raspberry Pi ------*/
  Serial.println("check1");        // send first startup check signal to pi
  while(Serial.available()==0) {   // Wait for pi to find arduino serial port and respond
  }
  Serial.read();                             // clear serial port
  digitalWrite(led,!digitalRead(led));       // turn on led
  Serial.println("check2");                  // send second startup check signal to pi

  while(Serial.available()==0) {             // pi startup check hands off to PiControl.py
  }                                          // wait for PiControl to respond
  int FrameSize = Serial.parseInt();         // receive value
  if (Serial.available()>0) {                // to speed up parseint, a terminator is added (eg. 100;)
    Serial.read();                           // Read terminator
  }
  digitalWrite(led,!digitalRead(led));       // turn off led, startup completed
  FrameSize = FrameSize;                     // do something with received value


  /*-------- Initialize and arm Motor -------*/
  Wire.beginTransmission(solSlave);      // Return Servos to Zero position
  Wire.write(253);
  Wire.endTransmission();

  digitalWrite(sol,HIGH);                // Thruster pulse check
  delay(soldelay);
  digitalWrite(sol,LOW);

  analogWrite(motorPin, pwmArm);         // initialize ESC at 0 Throttle
  delay(10000);                          // safety delay
  analogWrite(motorPin,pwmNom);          // Drive motor to nominal rpm
  delay(10000);                          // Wheel takes some time to reach nominal rpm

//  Wire.beginTransmission(solSlave);    // Input 30 pulsed angular impulse
//  Wire.write(30);
//  Wire.endTransmission();

}

/*===== Main Loop =====*/
void loop() {
  /*----- Servo check & rotation -----*/
  rpm = getTach();
  if (rpm < rpmNom) {                   // Wheel will need to recharge
    Wire.beginTransmission(solSlave);   // Turn servos to zero (thrust pointed counter clockwise)
    Wire.write(253);
    Wire.endTransmission();
    servocheck = false;                 // change position indicator
  }
  else {                                // Wheel will need to dump
    Wire.beginTransmission(solSlave);   // Turn servos to 180 (thrust pointed clockwise)
    Wire.write(254);
    Wire.endTransmission();
    servocheck = true;                  // change position indicator
  }

  /*---------- Dumping and Recharging -----------*/
  if (rpm > rpmThreshHigh) {
//    if ( gyroValue < gyro_thresh_bot ) {    // Regain control if system is still spinning when rpm threshold is reached
//      analogWrite(motorPin,119);            // ** Electronic interference affecting solenoid actuation
//      Wire.beginTransmission(solSlave);
//      Wire.write(1);
//      Wire.endTransmission();
//      while (gyroValue < gyro_thresh_bot) {
//          gyroValue = getGyro();
//      }
//      Wire.beginTransmission(solSlave);
//      Wire.write(2);
//      Wire.endTransmission();
//      delay(100);
//    }

    rpm = getTach();
    if(servocheck == false) {             // make sure servos are pointed in the correct direction
      Wire.beginTransmission(solSlave);   // Turn servos to 180 (thrust pointed clockwise)
      Wire.write(254);
      Wire.endTransmission();
      servocheck = true;
    }
    while ( rpm > 10500 ) {               // while rpm is greater than top of nominal threshold
//      Wire.beginTransmission(solSlave);  // ** Electronic interference affecting solenoid actuation
//      Wire.write(1);
//      Wire.endTransmission();
      digitalWrite(sol,HIGH);             // pulse once
      delay(soldelay);
      digitalWrite(sol,LOW);

      gyroValue = getGyro();
      while (gyroValue > gyro_thresh_top) {    // correct for pulse by decreasing rpm
        digitalWrite(sol,LOW);
        drive(gyroValue);
        gyroValue = getGyro();
      }
      rpm = getTach();
    }
  }
  else if (rpm < rpmThreshLow) {
//    if ( gyroValue > gyro_thresh_top ) {   // Regain control if system is still spinning when rpm threshold is reached
//      analogWrite(motorPin,107);            // ** Electronic interference affecting solenoid actuation
//      Wire.beginTransmission(solSlave);
//      Wire.write(1);
//      Wire.endTransmission();
//      while (gyroValue > gyro_thresh_top) {
//          gyroValue = getGyro();
//      }
//      Wire.beginTransmission(solSlave);
//      Wire.write(2);
//      Wire.endTransmission();
//      delay(100);
//    }

    rpm = getTach();
    if(servocheck == true) {              // make sure that servos are pointed in the correct direction
      Wire.beginTransmission(solSlave);   // Turn servos to zero (thrust pointed counter clockwise)
      Wire.write(253);
      Wire.endTransmission();
      servocheck = false;
    }
    while ( rpm < 10000 ) {                // while rpm is less than the bottom of the nominal rpm threshold
//      Wire.beginTransmission(solSlave);  // ** Electronic interference affecting solenoid actuation
//      Wire.write(1);
//      Wire.endTransmission();
      digitalWrite(sol,HIGH);              // pulse once
      delay(soldelay);
      digitalWrite(sol,LOW);

      gyroValue = getGyro();
      while (gyroValue < gyro_thresh_bot) {    // correct for pulse by increasing rpm
        digitalWrite(sol,LOW);
        drive(gyroValue);
        gyroValue = getGyro();
      }
      rpm = getTach();
    }
  }


  /*-------- Normal driving (operating between max/min rpm thresholds) --------*/
  gyroValue = getGyro();
  if (abs(gyroValue) > gyro_thresh_top) {
    drive(gyroValue);
  }
}



/*====== Get Tachometer ======*/
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

/*======= Get Gyro =======*/
double getGyro() {
  gyro.read();
  gyroStore[index_gyro] = gyro.data.z;
  index_gyro++;
  if (index_gyro > 3) {
    index_gyro = 0;
  }
  gyroSum = 0;
  for (int i=0; i<3; i++) {
   gyroSum = gyroSum + gyroStore[i];
  }
  gyroReading = gyroSum/3;
  return gyroReading;
}


/*==== Driving motor PID control ====*/
void drive(float gyroval) {
    rpm = getTach();
    rpmReading = rpm;

    gyro_calc = ((gyroval/6)*moi_frame/moi_fwa);
    rpmSet = rpmReading - gyro_calc;             // increase(or decrease) according to gyro calculation

    error = (rpmSet - rpmReading);
    error_tot = error_tot + error;

    error_moving[index] = error;
    index++;
    if (index > 9) {
      index = 0;
    }
    error_moving_tot = 0;
    for (int i = 0; i<10; i++) {
      error_moving_tot = error_moving_tot + error_moving[i];
    }
    error_moving_avg = error_moving_tot/10;


      P = kp*(error/rpmMax);
      I = ki*(error_tot/rpmMax);
      D = kd*((rpmReading - rpmLastRead)/rpmMax);

      scaler = (pwmMax-pwmMin)*((rpmSet-rpmMin)/rpmMax);
      Pdrive = scaler*P;
      Idrive = scaler*I;
      Ddrive = scaler*D;
      driveControl = pwmMin + scaler + Pdrive + Ddrive;

      if (driveControl > pwmLimit) {
        driveControl = pwmLimit;
      }
      else if (driveControl < pwmMin) {
        driveControl = pwmMin;
      }


      analogWrite(motorPin,driveControl);
      rpmLastRead = rpmReading;
      driveControl_old = driveControl;

      timereading = micros() - timeold;
      timeold = micros();

      Serial.print(timereading);
      Serial.print(", ");
      Serial.print(rpmReading);
      Serial.print(" , ");
      Serial.print(rpmSet);
      Serial.print(", ");
      Serial.print(driveControl);
      Serial.print(", ");
      Serial.println(gyroval);
}
