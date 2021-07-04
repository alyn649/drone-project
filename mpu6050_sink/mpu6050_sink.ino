#include <Arduino.h>
#include "DShot.h"

#define maxThrottle 400

// Objects for each motor
DShot motorA;
DShot motorB;
DShot motorC;
DShot motorD;

ISR(TIMER1_COMPA_vect) {
  DShot::callSendData(); // bit bang the motor commands
}


void setup() {
  Serial.begin(9600);

  // Attach each motor
  motorA.attach(6);
  motorB.attach(5);
  motorC.attach(4);
  motorD.attach(3);

  TCCR1B |= (1 << CS11) | (1 << CS10) | (1 << WGM12); // Prescaler of 64
  TCNT1 = 0;
  OCR1A = 1250; // results in 5.00ms delay
  TIMSK1 |= (1 << OCIE1A);

  sei();

  Serial.println("2 sec delay");
  delay(2000);

  Serial.print("Arming...");
  // Arm the motors
  motorA.setThrottle(48);
  motorB.setThrottle(48);
  motorC.setThrottle(48);
  motorD.setThrottle(48);
  delay(2000);
  Serial.println("Armed");

}

void loop() {
  Serial.println("Throttle up");
  for(int i = 48; i < maxThrottle; i++) {
    motorA.setThrottle(i);
    motorB.setThrottle(i);
    motorC.setThrottle(i);
    motorD.setThrottle(i);
    delay(15);
  }

  Serial.println("Hold");
  delay(5000);

  Serial.println("Throttle down");
  for(int i = maxThrottle; i >= 48; i--) {
    motorA.setThrottle(i);
    motorB.setThrottle(i);
    motorC.setThrottle(i);
    motorD.setThrottle(i);
    delay(15);
  }

  Serial.println("Off");
  while(1);
}
