#include <Servo.h>
#include <Arduino_RouterBridge.h>

Servo servo1;
Servo servo2;

void setup() {
  servo1.attach(10);
  servo2.attach(11);
  
  Bridge.begin();
  Bridge.provide("set_servo", set_servo);
  Bridge.provide("set_posture", set_posture);
}

void loop() {
  
}

void set_posture(int state, int angle) {
  // state: 0=Good, 1=Mid, 2=Bad
  // We move servo1 to indicate posture
  servo1.write(angle);
}

void set_servo(int num, int val)
{
  switch (num)
  {
    case 1:
      servo1.write(val);
      break;
    case 2:
      servo2.write(val);
      break;
    default:
      return;
  }
}