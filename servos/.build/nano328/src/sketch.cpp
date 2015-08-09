#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
void setup();
void sendData();
void receiveData(int checkwhatcame);
void loop();
#line 1 "src/sketch.ino"
//#include <Servo.h>
//#include <Wire.h>
#define slave_address 0x69

Servo servo1;
Servo servo2;
int val = 0;
int light = 5;
int led = 13;
int laser = 3;

boolean check = false;
boolean i2c_check_rec = true;
boolean i2c_check_send = true;

int servo_num = 0;
int servo_val = 90;

void setup()
{
	analogWrite(light,200);
	Wire.begin(slave_address);
	Serial.begin(115200);
	servo1.attach(9);
	servo2.attach(11);
	pinMode(laser,OUTPUT);	

	Wire.onRequest(sendData);
	Wire.onReceive(receiveData);
	delay(500);
}

void sendData() {
}

void receiveData(int checkwhatcame) {
	if (i2c_check_rec == true) {
		if (Wire.available()) {
			servo_num = Wire.read();
		}
		i2c_check_rec = false;
	}
	else {
		if (Wire.available()) {
			servo_val = Wire.read();
		}
		i2c_check_rec = true;
	}
	check = true;
}

void loop()
{
	if (check == true) {
		if (servo_num == 1) {
			servo1.write(servo_val);
		}
		else if (servo_num == 2) {
			servo2.write(servo_val);
		}
		else if (servo_num == 3) {
			digitalWrite(laser,!digitalRead(laser));
			servo_num = 0;
		}
	check = false;
	}
}
