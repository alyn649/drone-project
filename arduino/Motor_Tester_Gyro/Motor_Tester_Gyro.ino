#include <Arduino.h>
#include "DShot.h"
#include <TinyMPU6050.h>

#define maxThrottle 300

// Object for imu comms
MPU6050 imu (Wire);

// Objects for each motor
DShot motorA;
DShot motorB;
DShot motorC;
DShot motorD;

ISR(TIMER1_COMPA_vect) {
  DShot::callSendData(); // bit bang the motor commands
}

void setup() {
  Serial.begin(115200);

  // Await serial connection
  while(!Serial);

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

  //initilise imu
  imu.Initialize();

  imu.RegisterWrite(MPU6050_CONFIG, 0x03); // configure low pass filter

  //Serial.print("Calibrating...");
  imu.Calibrate();
  //Serial.println("done");

  //Serial.print("Arming...");
  // Arm the motors
  motorA.setThrottle(48);
  motorB.setThrottle(48);
  motorC.setThrottle(48);
  motorD.setThrottle(48);
  delay(2000);
  //Serial.println("Armed");

}

void loop() {
  //Serial.println("Throttle up");
  for(int i = 48; i < maxThrottle; i++) {
    motorA.setThrottle(i);
    motorB.setThrottle(i);
    motorC.setThrottle(i);
    motorD.setThrottle(i);
    delay(15);
  }

  //Serial.println("Hold");

  for(int i = 0; i < 1000; i++) {
    imu.Execute();
    Serial.print(imu.GetAccX());
    Serial.print(",");
    Serial.print(imu.GetAccY());
    Serial.print(",");
    Serial.print(imu.GetAccZ());
    Serial.print(",");
    Serial.print(imu.GetGyroX());
    Serial.print(",");
    Serial.print(imu.GetGyroY());
    Serial.print(",");
    Serial.print(imu.GetGyroZ());
    Serial.print(",");
    Serial.print(imu.GetAngX());
    Serial.print(",");
    Serial.print(imu.GetAngY());
    Serial.print(",");
    Serial.print(imu.GetAngZ());
    Serial.print("\r");
    
    delay(5);
  }
  

  //Serial.println("Throttle down");
  for(int i = maxThrottle; i >= 48; i--) {
    motorA.setThrottle(i);
    motorB.setThrottle(i);
    motorC.setThrottle(i);
    motorD.setThrottle(i);
    delay(15);
  }

  //Serial.println("Off");
  while(1);
}
