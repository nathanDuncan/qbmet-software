#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "CANHandler.h"
#include "Motor.h"
#include "RemoteDebug.h"

// --- CONFIGURATION ---

// CAN bus pins
#define CAN_TX 22
#define CAN_RX 21

// I2C pins
#define SDA_PIN 23
#define SCL_PIN 25

// PCA mux address (if using multiple MPUs)
#define PCA9548A_ADDR 0x70

// Motor & CAN handler
CANHandler canHandler;
Motor motor1(0x01, canHandler, Debug); // RIGHT HIP

// Preset "resting" gravity vector (measured when upright)
float gravity_resting[3] = {-0.981, 0.00, 0.00};;  // Replace with your actual upright gravity reading

// Filtered gravity state
float gravity_filtered[3] = {0, 0, 0};

// Low-pass filter parameter
const float alpha = 0.9;

// Gravity compensation gain
const float Kg = 10.0;

// MPU
Adafruit_MPU6050 mpu;

// Select I2C channel on multiplexer
void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Vector math
float vectorNorm(const float v[3]) {
  return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void normalize(float v[3]) {
  float norm = vectorNorm(v);
  if (norm == 0) return;
  for (int i = 0; i < 3; i++) v[i] /= norm;
}

void crossProduct(const float a[3], const float b[3], float result[3]) {
  result[0] = a[1]*b[2] - a[2]*b[1];
  result[1] = a[2]*b[0] - a[0]*b[2];
  result[2] = a[0]*b[1] - a[1]*b[0];
}

void lowPassFilter(const float input[3], float output[3]) {
  for (int i = 0; i < 3; i++) {
    output[i] = alpha * output[i] + (1 - alpha) * input[i];
  }
}

float computeGravityTorque(float gravity_current[3]) {
  float g1[3] = {gravity_resting[0], gravity_resting[1], gravity_resting[2]};
  float g2[3] = {gravity_current[0], gravity_current[1], gravity_current[2]};
  normalize(g1);
  normalize(g2);

  float cross[3];
  crossProduct(g1, g2, cross);
  float sin_theta = vectorNorm(cross);

  return Kg * sin_theta;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Debug.begin("GravityControl");

  Wire.begin(SDA_PIN, SCL_PIN);
  canHandler.setupCAN(CAN_TX, CAN_RX);

  motor1.start();
  motor1.reZero();

  // Init MPU (channel 0 for RIGHT HIP)
  selectMuxChannel(0);
  if (!mpu.begin()) {
    Serial.println("MPU0 not found!");
    while (1);
  }

  // Get initial accelerometer reading to initialize filter
  sensors_event_t accel_init, gyro_init, temp_init;
  mpu.getEvent(&accel_init, &gyro_init, &temp_init);

  gravity_filtered[0] = accel_init.acceleration.x;
  gravity_filtered[1] = accel_init.acceleration.y;
  gravity_filtered[2] = accel_init.acceleration.z;

  Serial.println("Filtered gravity initialized from first reading:");
  Serial.print("X: "); Serial.print(gravity_filtered[0], 2);
  Serial.print(" Y: "); Serial.print(gravity_filtered[1], 2);
  Serial.print(" Z: "); Serial.println(gravity_filtered[2], 2);
}

unsigned long lastStartTime = 0;
const unsigned long startInterval = 1000; 

void loop() {
  canHandler.update();
  motor1.update();

  // Re-start motor1 every 1 second
  unsigned long now = millis();
  if (now - lastStartTime >= startInterval) {
    motor1.start();
    lastStartTime = now;
  }

  // Get accelerometer reading
  selectMuxChannel(0);
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  float accel_raw[3] = {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z};
  lowPassFilter(accel_raw, gravity_filtered);

  // Compute torque based on difference from upright gravity vector
  float torque = computeGravityTorque(gravity_filtered);
  torque = constrain(torque, -19.0, 19.0);

  // Apply torque
  motor1.sendCommand(0, 0, 0, abs(torque) > 0.05 ? 0.6 : 0.0, torque);

  // Debug print
  Serial.print("sin(θ): "); Serial.print(torque / Kg, 3);
  Serial.print(" | torque: "); Serial.println(torque, 2);

  delay(10); // 100 Hz control loop
}

