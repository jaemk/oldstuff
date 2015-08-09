#include <Event.h>
#include <Timer.h>
#include <Wire.h>
#include <Adafruit_L3GD20.h>
#define tachSlave 1

Adafruit_L3GD20 gyro;
Timer t;
int motorPin = 11; 
int led = 13;

// I2C Transfer variables
int rpm = 0;
int result_rpm;
int left_rpm;
boolean printcheck = true;
byte buff = 0;

int PiCom = 0;
boolean serialcheck = false;
int timereading = 0;
int timeold = 0;

//MOI
float moi_frame = 0.03923;      //ALL MOI'S NEED TO BE VERIFIED *****
float moi_fwa = 9.2212e-5;
float rpm2rads = 0.1047;
double gyro_thresh = 1.00;
double gyroReading = 0;
float gyro_calc = 0;

int index_gyro = 0;
double gyroStore[3] = {0,0,0};
double gyroSum = 0;


// PID Drive Variables
double rpmReading = 0;
double rpmLastRead = 0;
double rpmSet = 0;
double val = 0;
double driveControl = 0;
double driveControl_old = 0;
double kp = 41;
double ki = 0;
double kd = 0;
double P = 0;
double I = 0;
double D = 0;
double scaler = 0;
double Pdrive = 0;
double Idrive = 0;
double Ddrive = 0;
double pwmLimit = 114;  // used to limit output, but not limit scale
double pwmMax = 125;
double pwmMin = 107;
double pwmNom = 107;
int pwmArm = 94;
double rpmMax = 26500;
double rpmMin = 5600;
double rpmNom = 10000;

double error = 0;
double error_tot = 0;
int error_moving[10] = {0,0,0,0,0,0,0,0,0,0};
double error_moving_tot = 0;
int index = 0;
double error_moving_avg = 0;

double rpmDrive = 0;
double rpmStore[11] = {0,0, 8000, 6500, 10000, 10500, 12000, 9000, 6000,9000,0};
int index_rpm = 0;

//int rpmstore[10] = { 16000,17000,16500,16300,16400,16300,17000,16000,15900,15500};
//int index2 = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  t.every(10000,changeDrive);
  gyro.begin(gyro.L3DS20_RANGE_250DPS); //initialize gyro
  pinMode(13,OUTPUT);
  digitalWrite(SDA,LOW);      // turn off internal I2C pull-up
  digitalWrite(SCL,LOW);      // turn off internal I2C pull-up
  
 
  //-------- Initialize and arm Motor -------//
  //analogWrite(motorPin,pwmArm);
  delay(10000);
  analogWrite(motorPin, pwmArm); // initialize ESC at 0 Throttle
  delay(10000); 
  analogWrite(motorPin,pwmNom);
  delay(5000);
//  rpmDrive = rpmStore[0];

}

void loop() {
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
  //Serial.println(gyroReading);
  
  if (abs(gyroReading) > gyro_thresh) {
    //Serial.println("poop");
    drive(gyroReading);
  }
  else {  
  }
 
}

void changeDrive() {
   index_rpm++;
   if(index_rpm > 11) {
     index_rpm = 0;
   }
   
   rpmDrive =  rpmStore[index_rpm];
}

void drive(float gyroval) {  
    rpm = getTach();
    rpmReading = rpm;
    //Serial.println((gyroval/6)*moi_frame/moi_fwa);
    
    gyro_calc = ((gyroval/6)*moi_frame/moi_fwa);
    //gyro_calc = (((gyroval * (60 / 360)) * moi_frame*255) / (moi_fwa));
    rpmSet = rpmReading - gyro_calc;   // increase(or decrease) according to gyro
    
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
  
//    if (abs(error_moving_avg) < 1) {
//      analogWrite(motorPin,driveControl_old);    
//    }
//    else {  
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
      Serial.print(gyro_calc);
      Serial.print(", ");
      Serial.println(gyroval);
      //gyro.read();
      
//      if (check == true) {
//        Serial.print(timereading);
//        Serial.print(", ");
//        Serial.print(rpmReading);
//        Serial.print(" , ");
//        Serial.print(rpmSet);
//        Serial.print(", ");
//        Serial.print(driveControl);
//        Serial.print(", ");
//        Serial.println(gyro.data.z);
//      }
    
    
  //}
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
