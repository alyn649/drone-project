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

int effortA;
int effortB;
int effortC;
int effortD;

int baseThrottle = 0;

uint16_t lastCycleTicks;

// Overload operators for xyzFloat to make control portion of program readable
xyzFloat operator+(xyzFloat a, xyzFloat b) {
  xyzFloat r;
  r.x = a.x + b.x;
  r.y = a.y + b.y;
  r.z = a.z + b.z;
  return r;
}

xyzFloat operator-(xyzFloat a, xyzFloat b) {
  xyzFloat r;
  r.x = a.x - b.x;
  r.y = a.y - b.y;
  r.z = a.z - b.z;
  return r;
}

xyzFloat operator*(xyzFloat a, float b) {
  xyzFloat r;
  r.x = a.x * b;
  r.y = a.y * b;
  r.z = a.z * b;
  return r;
}

xyzFloat operator*(xyzFloat a, double b) {
  xyzFloat r;
  r.x = a.x * b;
  r.y = a.y * b;
  r.z = a.z * b;
  return r;
}

xyzFloat operator*(xyzFloat a, xyzFloat b) {
  xyzFloat r;
  r.x = a.x * b.x;
  r.y = a.y * b.y;
  r.z = a.z * b.z;
  return r;
}

// Use PI control for gyro
xyzFloat gyrRef;
xyzFloat gyrErr;
xyzFloat gyrErrTot;
xyzFloat gyrKp;
xyzFloat gyrKi;
xyzFloat gyrEff;

// Use PI control for Acc
xyzFloat accRef;
xyzFloat accErr;
xyzFloat accErrTot;
xyzFloat accKp;
xyzFloat accKi;
xyzFloat accEff;

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
  imu.enableAccDLPF(false); // Disable low pass filter, allowing smallest delay possible

  // Control systems intial values

  // Gyro refernce
  gyrRef.x = 0;
  gyrRef.y = 0;
  gyrRef.z = 0;

  // Gyro intergral term
  gyrErrTot.x = 0;
  gyrErrTot.y = 0;
  gyrErrTot.z = 0;

  // Gyro kp
  gyrKp.x = 5;
  gyrKp.y = 5;
  gyrKp.z = 0.0;

  // Gyro ki
  gyrKi.x = 2;
  gyrKi.y = 2;
  gyrKi.z = 0;

  // Acc refernce
  accRef.x = 0;
  accRef.y = 0;
  accRef.z = 1;

  // Acc intergral term
  accErrTot.x = 0;
  accErrTot.y = 0;
  accErrTot.z = 0;

  // Acc kp
  accKp.x = 0;
  accKp.y = 0;
  accKp.z = 0;

  // Acc ki
  accKi.x = 10;
  accKi.y = 10;
  accKi.z = 0;

  // Setup timer for calculating control frequnecy
  TCCR1A = 0;
  TCCR1B = 0;
  
  TCCR1B |= (0<<CS12) | (1<<CS10);
  TCNT1 = 0;

  digitalWrite(13, LOW);
  _delay_ms(500);

}

void loop() {
    lastCycleTicks = TCNT1;
    TCNT1 = 0;
    acc = imu.getGValues();
    gyr = imu.getGyrValues();

//    if(Serial.available() > 0) {
//      baseThrottle = map(Serial.read(), 0, 255, 0, 1000);
//    }

    baseThrottle = map(analogRead(A0), 0, 1023, 0, 1000);
  
    // GYRO CONTROL SYSTEMS
    gyrErr = gyrRef - gyr; // Compute the error of the gyro
    
    gyrErrTot = gyrErrTot + (gyrErr * (lastCycleTicks * (double)0.0625 / 1000000)); // Intergrate error
    
    gyrEff = (gyrErr * gyrKp) + (gyrErrTot * gyrKi); // Compute control efforts for each axis
  
    // ACC CONTROL SYSTEM
    accErr = accRef - acc; // Compute the error of the acc
    
    accErrTot = accErrTot + (accErr * (lastCycleTicks * (double)0.0625 / 1000000)); // Intergrate error
    
    accEff = (accErr * accKp) + (accErrTot * accKi); // Compute control efforts for each axis
  
    // Apply 6 axis efforts to 4 motors accordingly
  
    effortA = baseThrottle - accEff.z + accEff.x - accEff.y - gyrEff.z - gyrEff.x - gyrEff.y;
    effortB = baseThrottle - accEff.z - accEff.x - accEff.y + gyrEff.z - gyrEff.x + gyrEff.y;
    effortC = baseThrottle - accEff.z + accEff.x + accEff.y + gyrEff.z + gyrEff.x - gyrEff.y;
    effortD = baseThrottle - accEff.z - accEff.x + accEff.y - gyrEff.z + gyrEff.x + gyrEff.y;
  
    // Saturate and step into correct range for dshot
    effortA = (sat(effortA, 0, EFFORT_SATURATION))*(baseThrottle > 5) + 48;
    effortB = (sat(effortB, 0, EFFORT_SATURATION))*(baseThrottle > 5) + 48;
    effortC = (sat(effortC, 0, EFFORT_SATURATION))*(baseThrottle > 5) + 48;
    effortD = (sat(effortD, 0, EFFORT_SATURATION))*(baseThrottle > 5) + 48;
    
    // Update the throttle values
    motorA.setThrottle(effortA);
    motorB.setThrottle(effortB);
    motorC.setThrottle(effortC);
    motorD.setThrottle(effortD);
//
//    Serial.print(" accx:");
//    Serial.print(accErr.x);
//    Serial.print(" accy:");
//    Serial.print(accErr.y);
//    Serial.print(" velx:");
//    Serial.print(accErrTot.x);
//    Serial.print(" vely:");
//    Serial.println(accErrTot.y);
//
//    Serial.print(" effx:");
//    Serial.print(accEff.x);
//    Serial.print(" effy:");
//    Serial.println(accEff.y);

    Serial.print("gyroy:");
    Serial.print(gyrErr.y);
    Serial.print(" gyrototy:");
    Serial.print(gyrErrTot.y);
    Serial.print(" gyroeffy:");
    Serial.print(gyrEff.y);
//    Serial.print(" accy:");
//    Serial.print(accErr.y);
//    Serial.print(" acctoty:");
//    Serial.print(accErrTot.y);
    Serial.print(" acceffy:");
    Serial.print(accEff.y);

    Serial.print(" effy:");
    Serial.println(gyrEff.y + accEff.y);

//    Serial.print(" effa:");
//    Serial.print(effortA);
//    Serial.print(" effb:");
//    Serial.print(effortB);
//    Serial.print(" effc:");
//    Serial.print(effortC);
//    Serial.print(" effd:");
//    Serial.println(effortD);  

    DShot::callSendData();
}

int sat(int val, int low, int up) {
  if(val < low) {
    val = low;
  }
  if(val > up) {
    val = up;
  }
  return val;
}
