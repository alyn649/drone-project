#include <MPU9250_WE.h>
#include <Wire.h>
#include "DShot.h"
#include <util/delay.h>

#define EFFORT_SATURATION 1200

MPU9250_WE imu = MPU9250_WE(0x68);

DShot motorA;
DShot motorB;
DShot motorC;
DShot motorD;

uint16_t effortA;
uint16_t effortB;
uint16_t effortC;
uint16_t effortD;

uint16_t lastCycleTicks;

bool lineEnded;
uint8_t targByte;

xyzFloat acc;
xyzFloat gyr;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(13, OUTPUT);

  digitalWrite(13, HIGH);

  motorA.attach(2);
  motorB.attach(3);
  motorC.attach(4);
  motorD.attach(5);

  motorA.setThrottle(0);
  motorB.setThrottle(0);
  motorC.setThrottle(0);
  motorD.setThrottle(0);

  _delay_ms(1000);
  imu.autoOffsets();

  // GYRO CONFIG
  imu.enableGyrDLPF(); // Enable low pass filer
  imu.setGyrDLPF(MPU9250_DLPF_6); // 
  imu.setSampleRateDivider(1); // 
  imu.setGyrRange(MPU9250_GYRO_RANGE_250); // Max range +- 250 d/s

  // ACC CONFIG
  imu.setAccRange(MPU9250_ACC_RANGE_2G); // Max range +- 2g
  imu.enableAccDLPF(true); // Disable low pass filter, allowing smallest delay possible
  imu.setAccDLPF(MPU9250_DLPF_6);


  // Setup timer for calculating control frequnecy
  TCCR1A = 0;
  TCCR1B = 0;
  
  TCCR1B |= (0<<CS12) | (1<<CS10);
  TCNT1 = 0;

  digitalWrite(13, LOW);

}

void loop() {
    lastCycleTicks = TCNT1;
    TCNT1 = 0;
    acc = imu.getGValues();
    gyr = imu.getGyrValues();

    // Send control period and 6 axis imu upstream
    Serial.print(lastCycleTicks);
    Serial.print(",");
    Serial.print(acc.x);
    Serial.print(",");
    Serial.print(acc.y);
    Serial.print(",");
    Serial.print(acc.z);
    Serial.print(",");
    Serial.print(gyr.x);
    Serial.print(",");
    Serial.print(gyr.y);
    Serial.print(",");
    Serial.println(gyr.z);

    // Wait for 8 byte packet downstream
    lineEnded = false;
    byte throttle[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    targByte = 0;
    
    while(!lineEnded) {
      // Wait for a byte
      if(Serial.available() > 0) {
        // Buff
        byte buff = Serial.read();
        // End on CR
        if(buff == '\r') {
          lineEnded = true;
        } else {
          throttle[targByte] = buff;
          targByte++;
        }
      }
    }

    // Shift bits to combine bytes into full 16bit range.
    effortA = ((uint16_t)throttle[0]<<8) | throttle[4];
    effortB = ((uint16_t)throttle[1]<<8) | throttle[5];
    effortC = ((uint16_t)throttle[2]<<8) | throttle[6];
    effortD = ((uint16_t)throttle[3]<<8) | throttle[7];

    // Update the throttle values
    motorA.setThrottle(effortA);
    motorB.setThrottle(effortB);
    motorC.setThrottle(effortC);
    motorD.setThrottle(effortD);

//    Serial.print(effortA);
//    Serial.print(",");
//    Serial.print(effortB);
//    Serial.print(",");
//    Serial.print(effortC);
//    Serial.print(",");
//    Serial.println(effortD);
    
    DShot::callSendData();
}
