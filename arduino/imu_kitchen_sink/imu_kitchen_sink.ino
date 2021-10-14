#include <MPU9250_WE.h>
#include <Wire.h>

MPU9250_WE imu = MPU9250_WE(0x68);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Start all 9 axis
  imu.init();
  imu.initMagnetometer();

  delay(1000);
  imu.autoOffsets(); // Get current pos as 0

  // Setup Gyro
  imu.enableGyrDLPF();
  imu.setGyrDLPF(MPU9250_DLPF_4);
  imu.setGyrRange(MPU9250_GYRO_RANGE_250);

  // Setup Acc
  imu.setAccRange(MPU9250_ACC_RANGE_2G);
  imu.enableAccDLPF(true);
  imu.setAccDLPF(MPU9250_DLPF_4); // 6 is most, but slowest

  imu.setMagOpMode(AK8963_CONT_MODE_8HZ);
  delay(100);
}

void loop() {
  xyzFloat acc = imu.getGValues();
  xyzFloat gyr = imu.getGyrValues();
  xyzFloat mag = imu.getMagValues();
  
  Serial.print("Acc (g): [");
  Serial.print(acc.x);
  Serial.print(" ");
  Serial.print(acc.y);
  Serial.print(" ");
  Serial.print(acc.z);
  
  Serial.print("], Gyro (D/s): [");
  Serial.print(gyr.x);
  Serial.print(" ");
  Serial.print(gyr.y);
  Serial.print(" ");
  Serial.print(gyr.z);

  Serial.print("], Mag (µT): ");
  Serial.print(mag.x);
  Serial.print(" ");
  Serial.print(mag.y);
  Serial.print(" ");
  Serial.print(mag.z);
  Serial.println("];");

  delay(100);
}
