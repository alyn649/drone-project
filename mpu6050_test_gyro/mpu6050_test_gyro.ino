#include <Arduino.h>
#include <TinyMPU6050.h>

MPU6050 mpu (Wire);

float kp_angle = 0.5;

float kp_rate = 0.25;
float ki_rate = 1.25;

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

int base = 0;

int throttle_a;
int throttle_b;
int throttle_c;
int throttle_d;

void setup() {
  mpu.Initialize();

  // Calibration
  Serial.begin(115200);
  mpu.Calibrate();
}

void loop() {
  mpu.Execute();

  rate_x = mpu.GetGyroX();
  rate_y = mpu.GetGyroY();
  rate_z = mpu.GetGyroZ();

  angle_x = mpu.GetAngX();
  angle_y = mpu.GetAngY();
  angle_z = mpu.GetAngZ();

  // Angle controller
  rate_x_ref = kp_angle * (angle_x_ref - angle_x);
  rate_y_ref = kp_angle * (angle_y_ref - angle_y);
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
  rate_x_eff = (kp_rate * rate_x_err) + (ki_rate * rate_x_err_sum);
  rate_y_eff = (kp_rate * rate_y_err) + (ki_rate * rate_y_err_sum);
  rate_z_eff = (kp_rate * rate_z_err) + (ki_rate * rate_z_err_sum);

  // Debuger
  Serial.print("angle_x:");
  Serial.print(angle_x);
  Serial.print(" angle_y:");
  Serial.print(angle_y);

  // Throttle conversion
  throttle_a = base - rate_y_eff + rate_x_eff;
  throttle_b = base - rate_y_eff - rate_x_eff;
  throttle_c = base + rate_y_eff + rate_x_eff;
  throttle_d = base + rate_y_eff - rate_x_eff;

  Serial.print(" throttle_a:");
  Serial.print(throttle_a);
  Serial.print(" throttle_b:");
  Serial.print(throttle_b);
  Serial.print(" throttle_c:");
  Serial.print(throttle_c);
  Serial.print(" throttle_d:");
  Serial.println(throttle_d);

}
