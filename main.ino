#include "Wire.h"

const int MPU_ADDR = 0x68;

int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temperature;

float kalman_angle_x, kalman_angle_y;
float kalman_bias_x, kalman_bias_y;
float P[2][2] = {{1, 0}, {0, 1}};
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;

float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
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
    Wire.read(); Wire.read(); // Skip temperature
    gx_sum += (Wire.read() << 8 | Wire.read());
    gy_sum += (Wire.read() << 8 | Wire.read());
    gz_sum += (Wire.read() << 8 | Wire.read());

    delay(3); // Small delay for stability
  }

  accel_offset_x = ax_sum / samples;
  accel_offset_y = ay_sum / samples;
  accel_offset_z = az_sum / samples;
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

  calibrate(); // Call the calibration routine
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  accelerometer_x = (Wire.read() << 8 | Wire.read()) - accel_offset_x;
  accelerometer_y = (Wire.read() << 8 | Wire.read()) - accel_offset_y;
  accelerometer_z = (Wire.read() << 8 | Wire.read()) - accel_offset_z;
  temperature = Wire.read() << 8 | Wire.read();
  gyro_x = (Wire.read() << 8 | Wire.read()) - gyro_offset_x;
  gyro_y = (Wire.read() << 8 | Wire.read()) - gyro_offset_y;
  gyro_z = (Wire.read() << 8 | Wire.read()) - gyro_offset_z;

  float dt = 0.01;
  float accel_angle_x = atan2(accelerometer_y, accelerometer_z) * 180 / 3.14159265359;
  float accel_angle_y = atan2(accelerometer_x, accelerometer_z) * 180 / 3.14159265359;

  float gyro_rate_x = gyro_x / 131.0;
  float gyro_rate_y = gyro_y / 131.0;

  kalman_angle_x = complementary_filter(accel_angle_x, gyro_rate_x, dt, &kalman_angle_x);
  kalman_angle_y = complementary_filter(accel_angle_y, gyro_rate_y, dt, &kalman_angle_y);

  Serial.print("Tilt direction: ");
  
  if (kalman_angle_x > 10) {
    Serial.print("Tilted Forward ");
  } else if (kalman_angle_x < -10) {
    Serial.print("Tilted Backward ");
  }

  if (kalman_angle_y > 10) {
    Serial.print("Tilted Right ");
  } else if (kalman_angle_y < -10) {
    Serial.print("Tilted Left ");
  }

  if (abs(kalman_angle_x) <= 10 && abs(kalman_angle_y) <= 10) {
    Serial.print("Stable ");
  }

  Serial.print(" | Acceleration: ");
  
  if (accelerometer_x > 3000) {
    Serial.print("Accelerating Right ");
  } else if (accelerometer_x < -3000) {
    Serial.print("Accelerating Left ");
  }

  if (accelerometer_y > 3000) {
    Serial.print("Accelerating Forward ");
  } else if (accelerometer_y < -3000) {
    Serial.print("Accelerating Backward ");
  }

  if (abs(accelerometer_x) <= 3000 && abs(accelerometer_y) <= 3000) {
    Serial.print("Stable ");
  }

  Serial.println();
  delay(100);
}
