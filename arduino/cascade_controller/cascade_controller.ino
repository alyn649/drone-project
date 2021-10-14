#include <Arduino.h>
#include "DShot.h"
#include <TinyMPU6050.h>

#define base 400
#define maxT 600

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

double gx, gy, gz;
double ax, ay, az;

double kp1 = 2; // angle contorller

double kp = 0.7;
double ki = 0.02;
double kd = 0;

double axErr, ayErr, azErr;

double gxRef, gyRef, gzRef;
double gxErr, gyErr, gzErr;
double gxErrT, gyErrT, gzErrT;

double effX, effY, effZ;

int effA, effB, effC, effD;

void loop() {
  //Serial.println("Throttle up");
  for(int i = 48; i < base; i++) {
    motorA.setThrottle(i);
    motorB.setThrottle(i);
    motorC.setThrottle(i);
    motorD.setThrottle(i);
    delay(15);
  }

  //Serial.println("Hold");

  for(int i = 0; i < 1000; i++) {
    imu.Execute();

    // Control system

    // get imu values
    gx = imu.GetGyroX();
    gy = imu.GetGyroY();
    gz = imu.GetGyroZ();

    ax = imu.GetAngX();
    ay = imu.GetAngY();
    az = imu.GetAngZ();

    // calc angle error with 0 ref
    axErr = 0 - ax;
    ayErr = 0 - ay;
    azErr = 0 - az;

    // calc ref value for rate controller
    gxRef = (axErr * kp1);
    gyRef = (ayErr * kp1);
    gzRef = (azErr * kp1);

    // compute error with angle fed ref
    gxErr = gxRef - gx;
    gyErr = gyRef - gy;
    gzErr = gzRef - gz;

    // intergrate error
    gxErrT += gxErr;
    gyErrT += gyErr;
    gzErrT += gzErr;

    // calculate final effort
    effX = (gxErr * kp) + (gxErrT * ki);
    effY = (gyErr * kp) + (gyErrT * ki);
    effZ = (gzErr * kp) + (gzErrT * ki);

    // apply to motors
    effA = base - effX - effY;
    effB = base + effX - effY;
    effC = base - effX + effY;
    effD = base + effX + effY;

    motorA.setThrottle(effA);
    motorB.setThrottle(effB);
    motorC.setThrottle(effC);
    motorD.setThrottle(effD);

    // print
    Serial.print(effX);
    Serial.print(",");
    Serial.print(effY);
    Serial.print(",");
    Serial.print(effZ);
    Serial.print(",");

//    Serial.print(ax);
//    Serial.print(",");
//    Serial.print(ay);
//    Serial.print(",");
//    Serial.print(az);
//    Serial.print(",");
    
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.print(gz);
    Serial.print("\r");
    
    delay(5);
  }
  

  //Serial.println("Throttle down");
  for(int i = base; i >= 48; i--) {
    motorA.setThrottle(i);
    motorB.setThrottle(i);
    motorC.setThrottle(i);
    motorD.setThrottle(i);
    delay(15);
  }

  //Serial.println("Off");
  while(1);
}
