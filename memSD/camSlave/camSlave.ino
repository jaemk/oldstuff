#include <Wire.h>
#define slave_address 2
  
signed int dist = 0;
int led = 13;
byte buffer;
boolean i2c_check = true;
signed int dist_hold = 0;
int LSBrightside;
int MSBleftside;
  
void setup() {
  Serial.begin(115200);
  digitalWrite(led,LOW);
  Wire.begin(slave_address);       // Initialize I2C @ 2
  
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
  
  Wire.onRequest(sendData);        // When requested, send cam data to master motorcontroller

  Serial.println("check1");        // send first startup check signal to pi
  while(Serial.available()==0) {   // Wait for pi to find arduino serial port and respond
  }
  Serial.read();                        // clear serial port
  digitalWrite(led,!digitalRead(led));  // turn on led
  Serial.println("check2");             // send second startup check signal to pi

  while(Serial.available()==0) {        // pi startup check hands off to cam_new
  }                                     // wait for cam_new to respond
  int camFrameSize = Serial.parseInt();      // Cam_new specifies camera frame size
  if (Serial.available()>0) {                // to speed up parseint, a terminator is added (100;)
    Serial.read();                           // Read terminator
  }
  digitalWrite(led,!digitalRead(led));       // turn off led, startup completed
  dist = 1;                                  // Motor controller will not start until dist > 0
  Serial.println("check3");
  
 }
 
void loop() {
  while(Serial.available()==0) {       // wait for serial data from pi
  }
  dist = Serial.parseInt();            // parse integer
  digitalWrite(led,!digitalRead(led)); // indicate reading with led
  if (Serial.available() > 0) {    
    buffer = Serial.read();            // clear serial port of terminator(;)
  }
}
 
void sendData() {     // Write cam distance data on request
   if (i2c_check == true) {
     dist_hold = dist;                   // hold distance value
     LSBrightside = (dist_hold & 0xFF);  // send right 8 bits
     Wire.write(LSBrightside);
     i2c_check = false;
   }
   else {
     MSBleftside = (dist_hold >> 8);    // sned left 8 bits
     Wire.write(MSBleftside);
     i2c_check = true;
   }
}
