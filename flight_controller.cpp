#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servoX;  // pin 9
Servo servoY;  // pin 10

// params
float kp = 0.20; 
float ki = 0.05; 
float kd = 0.005;

// pitch
float integralX = 0;
float lastErrorX = 0;

// roll
float integralY = 0;
float lastErrorY = 0;

unsigned long lastTime = 0;

// min max mid servo angle
const int servoXMin = 120;
const int servoXMid = 135;
const int servoXMax = 180;

const int servoYMin = 120;
const int servoYMid = 140;
const int servoYMax = 170;

// last pitch and roll for error
float lastPitch = 0.0;
float lastRoll = 0.0;

bool firstRun = true;

// low-pass filtered angles
float filteredPitch = 0.0;
float filteredRoll = 0.0;
const float alpha = 0.94;  // smoothing factor

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("mpu connection fail");
    while (1);
  }

  servoX.attach(9);
  servoY.attach(10);

  // Initialize servos to midpoint
  servoX.write(servoXMid);
  servoY.write(servoYMid);

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) return;
  lastTime = now;

  int16_t ax_raw, ay_raw, az_raw;
  mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);

  float ax = ax_raw / 16384.0;
  float ay = ay_raw / 16384.0;
  float az = az_raw / 16384.0;

  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

  // low-pass filtering
  filteredPitch = alpha * filteredPitch + (1 - alpha) * pitch;
  filteredRoll = alpha * filteredRoll + (1 - alpha) * roll;

  // pid pitch
  float errorX = 0.0 - filteredPitch;
  integralX += errorX * dt;
  float derivativeX = (errorX - lastErrorX) / dt;
  lastErrorX = errorX;

  float outputX = kp * errorX + ki * integralX + kd * derivativeX;

  // pid roll
  float errorY = 0.0 - filteredRoll;
  integralY += errorY * dt;
  float derivativeY = (errorY - lastErrorY) / dt;
  lastErrorY = errorY;

  float outputY = kp * errorY + ki * integralY + kd * derivativeY;

  // deadzone
  if (abs(errorX) < 1.0) {
    outputX = 0;
    integralX = 0;
  }
  if (abs(errorY) < 1.0) {
    outputY = 0;
    integralY = 0;
  }

  // sensitivity multiplier
  outputX *= 5.0;
  outputY *= 5.0;

  int servoXPos = constrain(servoXMid + (int)outputY, servoXMin, servoXMax);
  int servoYPos = constrain(servoYMid + (int)outputX, servoYMin, servoYMax);

  servoX.write(servoXPos);
  servoY.write(servoYPos);

  // for debug
  Serial.print("pitch: ");
  Serial.print(filteredPitch, 2);
  Serial.print(" | roll: ");
  Serial.print(filteredRoll, 2);
  Serial.print(" | ServoX (roll): ");
  Serial.print(servoXPos);
  Serial.print(" | ServoY (pitch): ");
  Serial.println(servoYPos);

  delay(10);
}
