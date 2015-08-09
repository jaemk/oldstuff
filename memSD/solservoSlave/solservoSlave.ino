
/* ==== Servo & Solenoid Slave ====*/

/* I2C bus Pull up resistors
   Using: 2k2 scl (a5-a5)
          4k7 sda (a4-a4)
          Pull-ups to 5v    */

#include <Servo.h>
#include <Wire.h>
#define slave_address 3

int led = 13;
int received_command = 5;         // 2,3 (left,right) rotate servo; 0,1 (off,on) solenoid state; 5 is null
boolean servo_check = true;        // servo-desired starting pos right
boolean pulse_check = false;      // solenoids off
boolean rot_check = true;        // servo-actual current pos right
Servo servo1;
Servo servo2;
int solpin = 5;              // solenoid actuation pin
int soldelay = 15;           // delay for solenoid actuation
int servoLimit = 180;        // servo max degree
int servoDelay = 5;          // servo step delay
int pulseval = 0;

void setup() {
  // Serial.begin(115200);
  pinMode(solpin,OUTPUT);
  pinMode(led,OUTPUT);
  digitalWrite(solpin,LOW);
  servo1.attach(9);
  servo2.attach(11);
  for (int i = 1; i < servoLimit; i++) {
    servo1.write(servoLimit);
    servo2.write(servoLimit);
    delay(servoDelay);
  }
  Wire.begin(slave_address);

  //--- Disable interal pullup resistors on I2C pins
    #ifndef cbi
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
    #endif
    #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
      // deactivate internal pull-ups for twi
      // as per note from atmega8 manual pg167
      cbi(PORTC, 4);
      cbi(PORTC, 5);
    #else
      // deactivate internal pull-ups for twi
      // as per note from atmega128 manual pg204
      cbi(PORTD, 0);
      cbi(PORTD, 1);
    #endif
  //---------------------------------------------------

  Wire.onReceive(receiveData);
}



// CALLBACK
void receiveData(int checkwhatwegot) {          // Receive commands
  if(Wire.available()){
    int val = Wire.read();
    //Serial.println(val);
    if (val < 253) {
     // pulseval = val;
     // pulse_check = true;
      if (val == 1) {          // =1 start pulsing
        pulse_check = true;
      }
      else {                  // = 0 stop pulsing
        pulse_check = false;
      }
    }
    else {
      if(val == 254) {          // turn to 180 clockwise
        servo_check = true;
      }
      else {                  // turn to 0 counter clockwise
        servo_check = false;
      }
    }
  }
}




// MAIN
void loop(){
  if (pulse_check == true) {
    //for (int i=0; i>pulseval; i++) {
      digitalWrite(solpin,HIGH);
      delay(soldelay);
      digitalWrite(solpin,LOW);
      delay(soldelay);
    //}
    //pulse_check == false;
  }
  else {
    digitalWrite(solpin,LOW);
  }
  
  if (servo_check==true) {       //want to turn to 180
    if (rot_check == false) {    // isn't already at 180
      //for (int i = 1; i < servoLimit; i++) {
        //servo1.write(i);
        //servo2.write(i);
        //delay(servoDelay);
      //}
      servo1.write(servoLimit);
      servo2.write(servoLimit);
      rot_check = true;          // is now at 180
    }
    else {
    }
  }
  else {                      // want to turn to 0
    if (rot_check == true) {  // isn't already at 0
//      for (int i = 178; i > 0; i--) {
//        servo1.write(i);
//        servo2.write(i);
//        delay(servoDelay);
//      }
      servo1.write(0);
      servo2.write(0);    
      rot_check = false;       // is not at 0
    }
    else {
    }
  }

}
