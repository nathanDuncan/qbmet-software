#include <Arduino.h>
#include <Wire.h>
#include "CANHandler.h"
#include "Motor.h"
#include "RemoteDebug.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// CAN TX / RX pins
static const int CAN_TX_PIN = 22;
static const int CAN_RX_PIN = 21;

static const int I2C_SDA_PIN = 23;
static const int I2C_SCL_PIN = 25;

// Global CAN handler
CANHandler canHandler;

// Motors
Motor motor1(0x01, canHandler, Debug); // RIGHT HIP
Motor motor2(0x02, canHandler, Debug); // RIGHT KNEE
Motor motor3(0x03, canHandler, Debug); // LEFT HIP
Motor motor4(0x04, canHandler, Debug); // LEFT KNEE

// MPU axis unit vectors - determined experimentally
float mpuVec1[3] = {0.25246345, 0.92360996, 0.28845599};     // channel 0 - right hip
float mpuVec2[3] = {-0.1785347, 0.73366519, 0.65563767};     // channel 1 - right knee
float mpuVec3[3] = {-0.26444315, 0.78469925, -0.56063973};   // channel 4 - left hip
// TODO: Recollect data for left knee PCA component
float mpuVec4[3] = {-0.47553835, 0.80868328, -0.34625804};   // channel 5 - left knee

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

// Proportional and Derivative gains
const float Kp_HIP = 4.5;
const float Kp_KNEE = 2.5;
const float Kd_HIP = 0.3;
const float Kd_KNEE = 0.15;
const float dt = 0.01; // time step in seconds (10ms = 100Hz)

float prev_omega0 = 0.0, prev_omega1 = 0.0, prev_omega4 = 0.0, prev_omega5 = 0.0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Beginning program...");

  Debug.begin("ESP32_Motor_Controller");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // CAN init
  canHandler.setupCAN(CAN_TX_PIN, CAN_RX_PIN);
  Serial.println("CAN bus initialized.");

  // Start and zero motors
  motor1.start(); motor2.start(); motor3.start(); motor4.start();
  motor1.reZero(); motor2.reZero(); motor3.reZero(); motor4.reZero();
  Serial.println("Motors re-zeroed.");

  // Initialize MPUs
  selectMuxChannel(0);
  if (!mpu.begin()) Serial.println("MPU0 not found!");
  delay(200);
  selectMuxChannel(4);
  if (!mpu.begin()) Serial.println("MPU4 not found!");
}

void loop() {
  canHandler.update();
  motor1.update(); motor2.update(); motor3.update(); motor4.update();

  // RIGHT HIP
  selectMuxChannel(0);
  sensors_event_t accel0, gyro0, temp0;
  mpu.getEvent(&accel0, &gyro0, &temp0);
  float gyro0vec[3] = {gyro0.gyro.x, gyro0.gyro.y, gyro0.gyro.z};
  float omega0 = dotProduct(gyro0vec, mpuVec1);
  float dOmega0 = (omega0 - prev_omega0) / dt;
  float torque1 = -Kp_HIP * omega0 - Kd_HIP * dOmega0;
  torque1 = constrain(torque1, -9.0, 9.0);
  motor1.sendCommand(0.0, 0.0, 0.0, abs(torque1) > 0.01 ? 0.6 : 0.0, abs(torque1) > 0.01 ? torque1 : 0.0);
  prev_omega0 = omega0;

  // RIGHT KNEE
  selectMuxChannel(1);
  sensors_event_t accel1, gyro1, temp1;
  mpu.getEvent(&accel1, &gyro1, &temp1);
  float gyro1vec[3] = {gyro1.gyro.x, gyro1.gyro.y, gyro1.gyro.z};
  float omega1 = dotProduct(gyro1vec, mpuVec2);
  float dOmega1 = (omega1 - prev_omega1) / dt;
  float torque2 = -Kp_KNEE * omega1 - Kd_KNEE * dOmega1;
  torque2 = constrain(torque2, -9.0, 9.0);
  motor2.sendCommand(0.0, 0.0, 0.0, abs(torque2) > 0.01 ? 0.6 : 0.0, abs(torque2) > 0.01 ? torque2 : 0.0);
  prev_omega1 = omega1;

  // LEFT HIP
  selectMuxChannel(4);
  sensors_event_t accel4, gyro4, temp4;
  mpu.getEvent(&accel4, &gyro4, &temp4);
  float gyro4vec[3] = {gyro4.gyro.x, gyro4.gyro.y, gyro4.gyro.z};
  float omega4 = dotProduct(gyro4vec, mpuVec3);
  float dOmega4 = (omega4 - prev_omega4) / dt;
  float torque3 = Kp_HIP * omega4 + Kd_HIP * dOmega4;
  torque3 = constrain(torque3, -9.0, 9.0);
  motor3.sendCommand(0.0, 0.0, 0.0, abs(torque3) > 0.01 ? 0.6 : 0.0, abs(torque3) > 0.01 ? torque3 : 0.0);
  prev_omega4 = omega4;

  // LEFT KNEE
  sensors_event_t accel5, gyro5, temp5;
  mpu.getEvent(&accel5, &gyro5, &temp5);
  float gyro5vec[3] = {gyro5.gyro.x, gyro5.gyro.y, gyro5.gyro.z};
  float omega5 = dotProduct(gyro5vec, mpuVec4);
  float dOmega5 = (omega5 - prev_omega5) / dt;
  float torque4 = Kp_KNEE * omega5 + Kd_KNEE * dOmega5;
  torque4 = constrain(torque4, -9.0, 9.0);
  motor4.sendCommand(0.0, 0.0, 0.0, abs(torque4) > 0.01 ? 0.6 : 0.0, abs(torque4) > 0.01 ? torque4 : 0.0);
  prev_omega5 = omega5;

  // Serial log
  Serial.print("omega0: "); Serial.print(omega0, 4);
  Serial.print(" | torque1: "); Serial.print(torque1, 4);
  Serial.print(" | omega1: "); Serial.print(omega1, 4);
  Serial.print(" | torque2: "); Serial.print(torque2, 4);
  Serial.print(" || omega4: "); Serial.print(omega4, 4);
  Serial.print(" | torque3: "); Serial.print(torque3, 4);
  Serial.print(" || omega5: "); Serial.print(omega5, 4);
  Serial.print(" | torque4: "); Serial.println(torque4, 4);

  delay(10); // 10ms loop = 100 Hz
}
