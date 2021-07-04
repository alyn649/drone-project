#include <Arduino.h>
#include <TinyMPU6050.h>
#include "DShot.h"

MPU6050 mpu (Wire);

DShot motorA;
DShot motorB;
DShot motorC;
DShot motorD;

float kp_angle_y = 0;

float kp_rate_y = 0.0;
float ki_rate_y = 0.0;

float kp_angle_x = 0;

float kp_rate_x = 0.0;
float ki_rate_x = 0.0;

float kp_angle = 4;

float kp_rate = 2;
float ki_rate = 0.01;

float rate_x;
float rate_y;
float rate_z;

float angle_x;
float angle_y;
float angle_z;

float angle_x_ref = 0;
float angle_y_ref = 0;
float angle_z_ref = 0;

float rate_x_ref;
float rate_y_ref;
float rate_z_ref;

float rate_x_err;
float rate_y_err;
float rate_z_err;

float rate_x_err_sum;
float rate_y_err_sum;
float rate_z_err_sum;

float rate_x_eff;
float rate_y_eff;
float rate_z_eff;

int baseMax = 350;

int base = 48;

uint32_t ticksRun = 0;

int throttle_a;
int throttle_b;
int throttle_c;
int throttle_d;

int kill = 900;

bool killFlag = false;

long timeStart;

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  mpu.Initialize();

  motorA.attach(6);
  motorB.attach(5);
  motorC.attach(4);
  motorD.attach(3);

  motorA.setThrottle(48);
  motorB.setThrottle(48);
  motorC.setThrottle(48);
  motorD.setThrottle(48);

  DShot::callSendData();

  digitalWrite(13, HIGH);

  // Calibration
  Serial.begin(115200);
  mpu.Calibrate();

  for(int i = 0; i < 100; i++) {
    motorA.setThrottle(48);
    motorB.setThrottle(48);
    motorC.setThrottle(48);
    motorD.setThrottle(48);
  
    DShot::callSendData();

    delay(10);
  }
  
  digitalWrite(13, LOW);
  timeStart = millis();
}

void loop() {
  if(Serial.available() > 0) {
    killFlag = true;
  }
  
  mpu.Execute();

  ticksRun++;
  
  if(base < baseMax) base++;

  rate_x = mpu.GetGyroX();
  rate_y = mpu.GetGyroY();
  rate_z = mpu.GetGyroZ();

  angle_x = mpu.GetAngX();
  angle_y = mpu.GetAngY();
  angle_z = mpu.GetAngZ();

  // Angle controller
  rate_x_ref = kp_angle_x * (angle_x_ref - angle_x);
  rate_y_ref = kp_angle_y * (angle_y_ref - angle_y);
  rate_z_ref = kp_angle * (angle_z_ref - angle_z);

  // Rate error
  rate_x_err = rate_x_ref - rate_x;
  rate_y_err = rate_y_ref - rate_y;
  rate_z_err = rate_z_ref - rate_z;

  // Intergrate rate error
  rate_x_err_sum += rate_x_err;
  rate_y_err_sum += rate_y_err;
  rate_z_err_sum += rate_z_err;
  
  // Rate controller
  rate_x_eff = (kp_rate_x * rate_x_err) + (ki_rate_x * rate_x_err_sum);
  rate_y_eff = (kp_rate_y * rate_y_err) + (ki_rate_y * rate_y_err_sum);
  rate_z_eff = (kp_rate * rate_z_err) + (ki_rate * rate_z_err_sum);

  // TEST SINGLE AXIS
  //rate_y_eff = 0;
  //rate_x_eff = 0;

//  if(ticksRun < 500) {
//    rate_y_eff = 0;
//    rate_x_eff = 0;
//  }

  // Throttle conversion
  throttle_a = int(base + rate_y_eff - rate_x_eff);
  throttle_b = int(base + rate_y_eff + rate_x_eff);
  throttle_c = int(base - rate_y_eff - rate_x_eff);
  throttle_d = int(base - rate_y_eff + rate_x_eff);

  // Ensure motors never send config mode values
  if(throttle_a < 48) throttle_a = 48;
  if(throttle_b < 48) throttle_b = 48;
  if(throttle_c < 48) throttle_c = 48;
  if(throttle_d < 48) throttle_d = 48;

  // if any try to excced the max throttle, kill the power
  if(throttle_a > kill | throttle_b > kill | throttle_c > kill | throttle_d > kill) {
    killFlag = true;
    //Serial.println("Kill From Over Throttle");
  }

//    Serial.print("throttle_a:");
//    Serial.print(throttle_a);
//    Serial.print(" throttle_b:");
//    Serial.print(throttle_b);
//    Serial.print(" throttle_c:");
//    Serial.print(throttle_c);
//    Serial.print(" throttle_d:");
//    Serial.print(throttle_d);

//    Serial.println();

  // If the kill flag has been raised, keep the motors off until reset
  if(killFlag) {
    throttle_a = 0;
    throttle_b = 0;
    throttle_c = 0;
    throttle_d = 0;
    digitalWrite(13, HIGH);
  } else {
     // Debuger

//      Serial.print("angle_eff_x:");
//      Serial.print(rate_x_ref);
//      Serial.print(" angle_eff_y:");
//      Serial.print(rate_y_ref);
//  
//      Serial.print(" gyro_eff_x:");
//      Serial.print(rate_x_eff);
//      Serial.print(" gyro_eff_y:");
//      Serial.print(rate_y_eff);
     
    Serial.print("angle_x:");
    Serial.print(angle_x);
//    Serial.print(" rate_x_ref:");
//    Serial.print(rate_x_ref);
//    Serial.print(" rate_x_err:");
//    Serial.print(rate_x_err);
//    Serial.print(" rate_x_eff:");
//    Serial.print(rate_x_eff);
    
    Serial.print(" angle_y:");
    Serial.print(angle_y);

    Serial.print(" gyro_x:");
    Serial.print(rate_x);
    Serial.print(" gyro_y:");
    Serial.print(rate_y);
    Serial.println();

  }

  

  // Update the throttle values in the dshot class
  motorA.setThrottle(throttle_a);
  motorB.setThrottle(throttle_b);
  motorC.setThrottle(throttle_c);
  motorD.setThrottle(throttle_d);

  // call the dshot class's static function to bit bang the throttles to the motor controller
  DShot::callSendData();



}
