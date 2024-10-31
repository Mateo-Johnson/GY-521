#include "Wire.h"

const int MPU_ADDR = 0x68;

int16_t acc_x, acc_y, acc_z;
int16_t rot_x, rot_y, rot_z;
int16_t temp;

float kAngle_x, kAngle_y;

float accoff_x = 0, accoff_y = 0, accoff_z = 0;
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;

char tmp_str[7];

char* convert_int16_to_str(int16_t i) {
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void calibrate() {
  int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
  int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
  const int samples = 1000;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    ax_sum += (Wire.read() << 8 | Wire.read());
    ay_sum += (Wire.read() << 8 | Wire.read());
    az_sum += (Wire.read() << 8 | Wire.read());
    Wire.read(); Wire.read();
    gx_sum += (Wire.read() << 8 | Wire.read());
    gy_sum += (Wire.read() << 8 | Wire.read());
    gz_sum += (Wire.read() << 8 | Wire.read());

    delay(3);
  }

  accoff_x = ax_sum / samples;
  accoff_y = ay_sum / samples;
  accoff_z = az_sum / samples;
  gyro_offset_x = gx_sum / samples;
  gyro_offset_y = gy_sum / samples;
  gyro_offset_z = gz_sum / samples;

  Serial.println("Calibration complete.");
}

float complementary_filter(float new_angle, float new_rate, float dt, float* angle) {
  *angle = 0.98 * (*angle + new_rate * dt) + 0.02 * new_angle;
  return *angle;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  calibrate(); 
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  acc_x = (Wire.read() << 8 | Wire.read()) - accoff_x;
  acc_y = (Wire.read() << 8 | Wire.read()) - accoff_y;
  acc_z = (Wire.read() << 8 | Wire.read()) - accoff_z;
  temp = Wire.read() << 8 | Wire.read();
  rot_x = (Wire.read() << 8 | Wire.read()) - gyro_offset_x;
  rot_y = (Wire.read() << 8 | Wire.read()) - gyro_offset_y;
  rot_z = (Wire.read() << 8 | Wire.read()) - gyro_offset_z;

  float dt = 0.01;
  float accel_angle_x = atan2(acc_y, acc_z) * 180 / 3.14159265359;
  float accel_angle_y = atan2(acc_x, acc_z) * 180 / 3.14159265359;

  float gyro_rate_x = rot_x / 131.0;
  float gyro_rate_y = rot_y / 131.0;

  kAngle_x = complementary_filter(accel_angle_x, gyro_rate_x, dt, &kAngle_x);
  kAngle_y = complementary_filter(accel_angle_y, gyro_rate_y, dt, &kAngle_y);

  Serial.print("Direction: ");
  
  if (kAngle_x > 10) {
    Serial.print("Tilted Forward (Angle: ");
    Serial.print(kAngle_x);
    Serial.print("°) ");
  } else if (kAngle_x < -10) {
    Serial.print("Tilted Backward (Angle: ");
    Serial.print(kAngle_x);
    Serial.print("°) ");
  }

  if (kAngle_y > 10) {
    Serial.print("Tilted Right (Angle: ");
    Serial.print(kAngle_y);
    Serial.print("°) ");
  } else if (kAngle_y < -10) {
    Serial.print("Tilted Left (Angle: ");
    Serial.print(kAngle_y);
    Serial.print("°) ");
  }

  Serial.print(" | Acceleration: ");
  
  if (acc_x > 3000) {
    Serial.print("Accelerating Right ");
  } else if (acc_x < -3000) {
    Serial.print("Accelerating Left ");
  }

  if (acc_y > 3000) {
    Serial.print("Accelerating Forward ");
  } else if (acc_y < -3000) {
    Serial.print("Accelerating Backward ");
  }

  if ((abs(kAngle_x) <= 10 && abs(kAngle_y) <= 10) && (abs(acc_x) <= 3000 && abs(acc_y) <= 3000)) {
    Serial.print("Stable ");
  }

  Serial.println();
  delay(100);
}
