#include <MPU6050.h>
#include <Wire.h>
MPU6050 mpu;

const int AIN1 = 2;
const int AIN2 = 3;
const int BIN1 = 4;
const int BIN2 = 7;
const int PWMA = 5;
const int PWMB = 6;
const int STBY = 8;

const int encoderA_MotorA = 9;
const int encoderB_MotorA = 10;
const int encoderA_MotorB = 11;
const int encoderB_MotorB = 12;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float gyroAngle = 0;
unsigned long lastTime = 0;
float angle = 0;

const float alpha = 0.98;

float Kp = 17;  // Proportional constant
float Ki = 1;   // Integral constant
float Kd = 2;   // Derivative constant

float last_error = 0;
float integral = 0;

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  lastTime = millis();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful!");
  } else {
    Serial.println("MPU6050 connection failed!");
  }

  calibrateGyro();

  delay(1000);
}

void calibrateGyro() {
  const int numSamples = 500;
  float sumX = 0;

  for (int i = 0; i < numSamples; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sumX += gx / 130.0;
    delay(3);
  }
}

void motorControl(float output) {
  int motorSpeed = abs(output);

  if (motorSpeed > 255) motorSpeed = 255;

  digitalWrite(STBY, HIGH);
  if (output > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMA, motorSpeed);
    analogWrite(PWMB, motorSpeed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMA, motorSpeed);
    analogWrite(PWMB, motorSpeed);
  }
}

void loop() {
  // mpu.getAcceleration(&ax, &ay, &az);
  // mpu.getRotation(&gx, &gy, &gz);

  // float accelAngle = atan2(ay, az) * 180.0 / PI;

  // float gyro_rate = gx / 131.0;

  // unsigned long currentTime = micros();
  // float dt = (currentTime - lastTime) / 1000000.0;
  // lastTime = currentTime;

  // angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accelAngle;

  // float error = -0.7 - angle;

  // integral += error;
  // float derivative = error - last_error;

  // float output = Kp * error + Ki * integral + Kd * derivative;

  // motorControl(output);
  // last_error = error;
}