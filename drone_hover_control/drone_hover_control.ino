#include <MPU9250_WE.h>
#include <Wire.h>
#include "DShot.h"

#define EFFORT_SATURATION 400

MPU9250_WE imu = MPU9250_WE(0x68);

DShot motorA;
DShot motorB;
DShot motorC;
DShot motorD;

int effortA;
int effortB;
int effortC;
int effortD;

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

void setup() {
  Wire.begin();

  motorA.attach(2);
  motorB.attach(3);
  motorC.attach(4);
  motorD.attach(5);

  motorA.setThrottle(0);
  motorB.setThrottle(0);
  motorC.setThrottle(0);
  motorD.setThrottle(0);

  delay(1000);
  imu.autoOffsets();

  // GYRO CONFIG
  imu.enableGyrDLPF(); // Enable low pass filer
  imu.setGyrDLPF(MPU9250_DLPF_7); // 0.17ms delay
  imu.setSampleRateDivider(0); // Keep sample rate same as internal sample rate
  imu.setGyrRange(MPU9250_GYRO_RANGE_250); // Max range +- 250 d/s

  // ACC CONFIG
  imu.setAccRange(MPU9250_ACC_RANGE_2G); // Max range +- 2g
  imu.enableAccDLPF(false); // Disable low pass filter, allowing smallest delay possible

  delay(100);

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
  gyrKp.x = 0.01;
  gyrKp.y = 0.01;
  gyrKp.z = 0.001;

  // Gyro ki
  gyrKi.x = 0.001;
  gyrKi.y = 0.001;
  gyrKi.z = 0.0001;


  // Acc refernce
  accRef.x = 0;
  accRef.y = 0;
  accRef.z = 1;

  // Acc intergral term
  accErrTot.x = 0;
  accErrTot.y = 0;
  accErrTot.z = 0;

  // Acc kp
  accKp.x = 1;
  accKp.y = 1;
  accKp.z = 100;

  // Acc ki
  accKi.x = 0.1;
  accKi.y = 0.1;
  accKi.z = 1;

}

// Superloop control system
void loop() {
  xyzFloat acc = imu.getGValues();
  xyzFloat gyr = imu.getGyrValues();

  // GYRO CONTROL SYSTEMS
  gyrErr = gyrRef - gyr; // Compute the error of the gyro
  
  gyrErrTot = gyrErrTot + gyrErr; // Intergrate error
  
  gyrEff = (gyrErr * gyrKp) + (gyrErr * gyrKi); // Compute control efforts for each axis

  // ACC CONTROL SYSTEM
  accErr = accRef - acc; // Compute the error of the acc
  
  accErrTot = accErrTot + accErr; // Intergrate error
  
  accEff = (accErr * accKp) + (accErr * accKi); // Compute control efforts for each axis

  // Apply 6 axis efforts to 4 motors accordingly

  effortA = accEff.z - accEff.x + accEff.y - gyrEff.z - gyrEff.x - gyrEff.y;
  effortB = accEff.z - accEff.x - accEff.y + gyrEff.z + gyrEff.x - gyrEff.y;
  effortC = accEff.z + accEff.x + accEff.y + gyrEff.z - gyrEff.x + gyrEff.y;
  effortD = accEff.z + accEff.x - accEff.y - gyrEff.z + gyrEff.x + gyrEff.y;

  // Saturate and step into correct range for dshot
  effortA = sat(effortA, 0, EFFORT_SATURATION) + 48;
  effortB = sat(effortB, 0, EFFORT_SATURATION) + 48;
  effortC = sat(effortC, 0, EFFORT_SATURATION) + 48;
  effortD = sat(effortD, 0, EFFORT_SATURATION) + 48;

  // Update the throttle values
  motorA.setThrottle(effortA);
  motorB.setThrottle(effortB);
  motorC.setThrottle(effortC);
  motorD.setThrottle(effortD);
  
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
