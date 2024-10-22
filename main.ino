#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

struct KalmanFilter {
  float Q_angle, Q_bias, R_measure;
  float angle, bias, rate;
  float P[2][2];

  KalmanFilter() {
    Q_angle = 0.001;
    Q_bias = 0.003;
    R_measure = 0.03;
    angle = 0;
    bias = 0;
    P[0][0] = 0; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = 0;
  }

  float getAngle(float newAngle, float newRate, float dt) {
    rate = newRate - bias;
    angle += dt * rate;
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
    return angle;
  }

  void setNoise(float processNoise, float measurementNoise) {
    Q_angle = processNoise;
    R_measure = measurementNoise;
  }
};

KalmanFilter kalmanX, kalmanY, kalmanZ;

float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

void calibrateSensors(int samples) {
  long sumAx = 0, sumAy = 0, sumAz = 0, sumGx = 0, sumGy = 0, sumGz = 0;
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumAx += ax; sumAy += ay; sumAz += az;
    sumGx += gx; sumGy += gy; sumGz += gz;
    delay(10);
  }
  accelBiasX = sumAx / samples;
  accelBiasY = sumAy / samples;
  accelBiasZ = sumAz / samples;
  gyroBiasX = sumGx / samples;
  gyroBiasY = sumGy / samples;
  gyroBiasZ = sumGz / samples;
}

float calculateTemperature() {
  int16_t rawTemp = mpu.getTemperature();
  return rawTemp / 340.0 + 36.53;
}

float calculateRoll(float accelX, float accelY, float accelZ) {
  return atan2(accelY, accelZ) * 180 / M_PI;
}

float calculatePitch(float accelX, float accelY, float accelZ) {
  return atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;
}

float calculateYaw(float gyroZ, float dt) {
  static float yaw = 0;
  yaw += (gyroZ - gyroBiasZ) * dt;
  return yaw;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1); 
  }
  calibrateSensors(500);
  kalmanX.setNoise(0.001, 0.03);
  kalmanY.setNoise(0.001, 0.03);
  kalmanZ.setNoise(0.001, 0.03);
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = (ax - accelBiasX) / 16384.0;
  float accelY = (ay - accelBiasY) / 16384.0;
  float accelZ = (az - accelBiasZ) / 16384.0;
  float gyroX = (gx - gyroBiasX) / 131.0;
  float gyroY = (gy - gyroBiasY) / 131.0;
  float gyroZ = (gz - gyroBiasZ) / 131.0;

  float dt = 0.01;
  float roll = calculateRoll(accelX, accelY, accelZ);
  float pitch = calculatePitch(accelX, accelY, accelZ);
  float yaw = calculateYaw(gyroZ, dt);

  float filteredRoll = kalmanX.getAngle(roll, gyroX, dt);
  float filteredPitch = kalmanY.getAngle(pitch, gyroY, dt);
  float filteredYaw = kalmanZ.getAngle(yaw, gyroZ, dt);

  float temperature = calculateTemperature();

  Serial.print("Roll: "); Serial.print(filteredRoll);
  Serial.print(", Pitch: "); Serial.print(filteredPitch);
  Serial.print(", Yaw: "); Serial.print(filteredYaw);
  Serial.print(", Temp: "); Serial.println(temperature);

  delay(10);
}
