/* ==== Tachometer Slave ====*/

/* I2C bus Pull up resistors
   Using: 2k2 scl (a5-a5)
          4k7 sda (a4-a4)
          Pull-ups to 5v    */

#include <Wire.h>
#define slave_address 1

volatile byte count = 0;
boolean check = false;
boolean i2c_check = true;
unsigned long timeold = 0;
unsigned long timereading = 0;
unsigned long timecount = 0;
long hold1 = 0;
long hold2 = 0;
long hold3 = 0;
long rpm_scale = 3 * 30 * (1000000);   // for two interrupts
long rpm = 0;
long rpm_hold = 0;
int LSBrightside;
int MSBleftside;

void setup() {
  Serial.begin(115200);
  // pinMode(13,OUTPUT);
  // digitalWrite(13,HIGH);
  Wire.begin(slave_address);          // Initialize as I2C slave @ 1
  
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
  
  Wire.onRequest(sendData);           // Define callback function for I2C requests
  attachInterrupt(0, tick, FALLING);  // Interrupt 0 is digital pin 2
}

void loop() {
  while (check == true) {                      // Wait for interrupt

    detachInterrupt(0);                     // Prevent interrupt during calculation
    timereading = micros() - timeold;       // Calculate time since previous interrupt
    timeold = micros();                     // Reset time reference
//      Serial.print(hold1);
//      Serial.print(" , ");
//      Serial.print(hold2);
//      Serial.print(" , ");
//      Serial.print(count);
//      Serial.print(" , ");
//      Serial.println(hold3);
    
    timecount = timecount  + timereading;   // Add to running total (3 readings)
  
    if (count == 3) {                    // Check if sample size is reached
      timecount = timecount - hold1;     // Subtract timereading value saved the last time count = 3
      hold1 = timereading;               // Save current timereading value for count = 3
      count = 0;                         // Reset count
    }
    else if (count == 2) {
      timecount = timecount - hold2;
      hold2 = timereading;
    }
    else if (count == 1) {
      timecount = timecount - hold3;
      hold3 = timereading;
    }
    else {                         // reset count in the case of interrupt bouncing over 3
      count = 0;
    }
    rpm = (rpm_scale/timecount);
    Serial.print(timereading);
    Serial.print(", ");
    Serial.println(rpm);
    check = false;
    attachInterrupt(0, tick, FALLING);
  }
}

void tick() {                     // Tachometer interrupt callback
  count++;
  check = true;
  // digitalWrite(13,!digitalRead(13));
}


void sendData() {                 // I2C wire request callback
  if (i2c_check == true) {        // Send LSB right side of 16 bit int
    rpm_hold = rpm;                    // hold rpm value for sending
    // Serial.println(rpm_hold);
    LSBrightside = (rpm_hold & 0xFF);  // pull right 8 bits
    Wire.write(LSBrightside);          // write right 8 bits
    i2c_check = false;                 // on next request, the left 8 bits will be sent
  }
  else {
    MSBleftside = (rpm_hold >> 8);
    Wire.write(MSBleftside);
    i2c_check = true;
  }
}

