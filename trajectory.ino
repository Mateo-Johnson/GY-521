#include "Wire.h"

const int MPU_ADDR = 0x68;
const float RAD_TO_DEG = 180 / 3.14159265359;

int16_t acc_x, acc_y, acc_z;
int16_t rot_x, rot_y, rot_z;
int16_t temp;
float kAngle_x, kAngle_y;
float accoff_x = 0, accoff_y = 0, accoff_z = 0;
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;

float pos_x = 0, pos_y = 0;
float vel_x = 0, vel_y = 0;
unsigned long lastTime = 0;

struct Point {
  float x;
  float y;
};

const int NUM_WAYPOINTS = 4;
Point waypoints[NUM_WAYPOINTS] = {
  {0, 0},
  {100, 0},
  {100, 100},
  {0, 100}
};

int currentWaypoint = 0;
const float WAYPOINT_THRESHOLD = 10.0;

char tmp_str[7];
char* convert_int16_to_str(int16_t i) {
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void calibrate() {
  int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
  int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
  const int samples = 1000;
  
  Serial.println("Calibrating...");
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

void updatePosition(float dt) {
  float acc_x_ms2 = (acc_x / 16384.0) * 9.81;
  float acc_y_ms2 = (acc_y / 16384.0) * 9.81;
  
  if (abs(acc_x_ms2) > 0.1) {
    vel_x += acc_x_ms2 * dt;
  }
  if (abs(acc_y_ms2) > 0.1) {
    vel_y += acc_y_ms2 * dt;
  }
  
  vel_x *= 0.95;
  vel_y *= 0.95;
  
  pos_x += vel_x * dt;
  pos_y += vel_y * dt;
}

float getDistanceToWaypoint() {
  float dx = waypoints[currentWaypoint].x - pos_x;
  float dy = waypoints[currentWaypoint].y - pos_y;
  return sqrt(dx * dx + dy * dy);
}

void checkWaypointProgress() {
  if (getDistanceToWaypoint() < WAYPOINT_THRESHOLD) {
    currentWaypoint = (currentWaypoint + 1) % NUM_WAYPOINTS;
    Serial.print("Reached waypoint! Moving to waypoint ");
    Serial.println(currentWaypoint);
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  calibrate();
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
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
  
  float accel_angle_x = atan2(acc_y, acc_z) * RAD_TO_DEG;
  float accel_angle_y = atan2(acc_x, acc_z) * RAD_TO_DEG;
  float gyro_rate_x = rot_x / 131.0;
  float gyro_rate_y = rot_y / 131.0;
  
  kAngle_x = complementary_filter(accel_angle_x, gyro_rate_x, dt, &kAngle_x);
  kAngle_y = complementary_filter(accel_angle_y, gyro_rate_y, dt, &kAngle_y);
  
  updatePosition(dt);
  checkWaypointProgress();
  
  Serial.print("Position (x,y): (");
  Serial.print(pos_x);
  Serial.print(", ");
  Serial.print(pos_y);
  Serial.print(") | Target: (");
  Serial.print(waypoints[currentWaypoint].x);
  Serial.print(", ");
  Serial.print(waypoints[currentWaypoint].y);
  Serial.print(") | Distance to target: ");
  Serial.println(getDistanceToWaypoint());
  
  Serial.print("Orientation: ");
  if (kAngle_x > 10) Serial.print("Forward ");
  else if (kAngle_x < -10) Serial.print("Backward ");
  if (kAngle_y > 10) Serial.print("Right ");
  else if (kAngle_y < -10) Serial.print("Left ");
  
  Serial.print("| Motion: ");
  if (abs(vel_x) > 0.5 || abs(vel_y) > 0.5) {
    if (vel_x > 0.5) Serial.print("Moving Right ");
    else if (vel_x < -0.5) Serial.print("Moving Left ");
    if (vel_y > 0.5) Serial.print("Moving Forward ");
    else if (vel_y < -0.5) Serial.print("Moving Backward ");
  } else {
    Serial.print("Stable ");
  }
  Serial.println();
  
  delay(50);
}
