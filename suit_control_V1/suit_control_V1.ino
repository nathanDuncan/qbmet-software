#include <Arduino.h>
#include <Wire.h>
#include "CANHandler.h"
#include "Motor.h"
#include "RemoteDebug.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// CAN TX / RX pins
static const int CAN_TX_PIN = 22;   // D22 = TX
static const int CAN_RX_PIN = 21;   // D21 = RX

static const int I2C_SDA_PIN = 23;
static const int I2C_SCL_PIN = 25;

// Global CAN handler
CANHandler canHandler;

// Motors
Motor motor1(0x01, canHandler, Debug); // RIGHT HIP
Motor motor2(0x02, canHandler, Debug); // RIGHT KNEE
Motor motor3(0x03, canHandler, Debug); // LEFT HIP
Motor motor4(0x04, canHandler, Debug); // LEFT KNEE


// MPU axis unit vectors - these were determined experimentally
float mpuVec1[3] = {0.25246345, 0.92360996, 0.28845599}; // channel 0 - right hip
float mpuVec2[3] = {-0.1785347   0.73366519  0.65563767}; // channel 1 - right knee
float mpuVec3[3] = {-0.26444315, 0.78469925, -0.56063973}; // channel 4 - left hip
float mpuVec4[3] = {-0.47553835  0.80868328 -0.34625804}; // channel 5 - left knee

Adafruit_MPU6050 mpu;

#define PCA9548A_ADDR 0x70

void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Dot product helper
float dotProduct(float gyroVec[3], float sensorVec[3]) {
  float sum = 0.0;
  for (int i = 0; i < 3; i++) {
    sum += gyroVec[i] * sensorVec[i];
  }
  return sum;
}

// Proportional gain for control
const float Kp_HIP = 4.5;
const float Kp_KNEE = 2.5;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Beginning program...");

  Debug.begin("ESP32_Motor_Controller");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // ---------- CAN BUS INITIALISATION ----------
  canHandler.setupCAN(CAN_TX_PIN, CAN_RX_PIN);
  Serial.println("CAN bus initialized.");

  // Initialise motors
  motor1.start();
  motor2.start();
  motor3.start();
  motor4.start();
  motor1.reZero();
  motor2.reZero();
  motor3.reZero();
  motor4.reZero();
  Serial.println("Motors re-zeroed.");

  // Initialize both MPUs
  selectMuxChannel(0);
  if (!mpu.begin()) {
    Serial.println("MPU0 not found!");
  }
  delay(200);
  selectMuxChannel(4);
  if (!mpu.begin()) {
    Serial.println("MPU4 not found!");
  }
}

void loop() {
  canHandler.update();
  motor1.update();
  motor2.update();
  motor3.update();
  motor4.update();

  // MPU0 (RIGHT HIP)
  selectMuxChannel(0);
  sensors_event_t accel0, gyro0, temp0;
  mpu.getEvent(&accel0, &gyro0, &temp0);
  float gyro0vec[3] = {gyro0.gyro.x, gyro0.gyro.y, gyro0.gyro.z};
  float omega0 = dotProduct(gyro0vec, mpuVec1);

  // MPU1 (RIGHT KNEE)
  selectMuxChannel(1);
  sensors_event_t accel1, gyro1, temp1;
  mpu.getEvent(&accel1, &gyro1, &temp1);
  float gyro1vec[3] = {gyro1.gyro.x, gyro1.gyro.y, gyro1.gyro.z};
  float omega1 = dotProduct(gyro1vec, mpuVec2);

  // MPU4 (LEFT HIP)
  selectMuxChannel(4);
  sensors_event_t accel4, gyro4, temp4;
  mpu.getEvent(&accel4, &gyro4, &temp4);
  float gyro4vec[3] = {gyro4.gyro.x, gyro4.gyro.y, gyro4.gyro.z};
  float omega4 = dotProduct(gyro4vec, mpuVec3);

  // MPU4 (LEFT KNEE)
  selectMuxChannel(4);
  sensors_event_t accel5, gyro5, temp5;
  mpu.getEvent(&accel5, &gyro5, &temp5);
  float gyro5vec[3] = {gyro5.gyro.x, gyro5.gyro.y, gyro5.gyro.z};
  float omega5 = dotProduct(gyro5vec, mpuVec4);

  float torque1 = 0.0;
  float torque2 = 0.0;
  float torque3 = 0.0;
  float torque4 = 0.0;

  // Threshold for both directions
  // RIGHT HIP
  if (abs(omega0) > 0.01) {
    torque1 = -Kp_HIP * omega0;
    torque1 = constrain(torque1, -9.0, 9.0);
    motor1.sendCommand(0.0, 0.0, 0.0, 0.6, torque1);
  } else {
    motor1.sendCommand(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  // RIGHT KNEE
  if (abs(omega1) > 0.01) {
    torque2 = -Kp_KNEE * omega1;
    torque2 = constrain(torque2, -9.0, 9.0);
    motor2.sendCommand(0.0, 0.0, 0.0, 0.6, torque2);
  } else {
    motor2.sendCommand(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  // LEFT HIP
  if (abs(omega4) > 0.01) {
    torque3 = Kp_HIP * omega4;
    torque3 = constrain(torque3, -9.0, 9.0);
    motor3.sendCommand(0.0, 0.0, 0.0, 0.6, torque3);
  } else {
    motor3.sendCommand(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  // LEFT KNEE
  if (abs(omega5) > 0.01) {
    torque4 = Kp_KNEE * omega5;
    torque4 = constrain(torque4, -9.0, 9.0);
    motor4.sendCommand(0.0, 0.0, 0.0, 0.6, torque4);
  } else {
    motor4.sendCommand(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  // Always print for log
  Serial.print("omega0: "); Serial.print(omega0, 4);
  Serial.print(" | torque1: "); Serial.print(torque1, 4);
  Serial.print("omega1: "); Serial.print(omega1, 4);
  Serial.print(" | torque2: "); Serial.print(torque2, 4);
  Serial.print(" || omega4: "); Serial.print(omega4, 4);
  Serial.print(" | torque3: "); Serial.println(torque3, 4);
  Serial.print(" || omega5: "); Serial.print(omega5, 4);
  Serial.print(" | torque4: "); Serial.println(torque4, 4);

  delay(10); // 20 Hz
}

