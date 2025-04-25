#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define SDA_PIN 23
#define SCL_PIN 25
#define PCA9548A_ADDR 0x70

/*
Principal axis (in sensor coordinates) in MPU #1 (MUX ID = 0): [0.25246345 0.92360996 0.28845599]
Principal axis (in sensor coordinates) in MPU #2 (MUX ID = 1): [-0.1785347   0.73366519  0.65563767]
Principal axis (in sensor coordinates) in MPU #3 (MUX ID = 4): [-0.26444315  0.78469925 -0.56063973]
Principal axis (in sensor coordinates) in MPU #4 (MUX ID = 5): [-0.47553835  0.80868328 -0.34625804]
*/

float mpuVec1[3] = {0.25246345, 0.92360996, 0.28845599};    // channel 0
float mpuVec3[3] = {-0.26444315, 0.78469925, -0.56063973};  // channel 4

// Dot product helper
float dotProduct(float gyroVec[3], float sensorVec[3]) {
  float sum = 0.0;
  for (int i = 0; i < 3; i++) {
    sum += gyroVec[i] * sensorVec[i];
  }
  return sum;
}

Adafruit_MPU6050 mpu;

void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(1000);

  // Initialize MPU on channel 0
  selectMuxChannel(0);
  if (!mpu.begin()) {
    Serial.println("MPU0 not found!");
  }
  delay(200);
  // Initialize MPU on channel 4
  selectMuxChannel(4);
  if (!mpu.begin()) {
    Serial.println("MPU4 not found!");
  }

  // Serial Plotter header
  Serial.println("omega0\tomega4\tgx0\tgy0\tgz0\tgx4\tgy4\tgz4");
}

void loop() {
  sensors_event_t accel0, gyro0, temp0;
  sensors_event_t accel4, gyro4, temp4;

  // Read from MPU 0 (channel 0)
  selectMuxChannel(0);
  mpu.getEvent(&accel0, &gyro0, &temp0);
  float gyro0vec[3] = {gyro0.gyro.x, gyro0.gyro.y, gyro0.gyro.z};
  float omega0 = dotProduct(gyro0vec, mpuVec1);

  // Read from MPU 4 (channel 4)
  selectMuxChannel(4);
  mpu.getEvent(&accel4, &gyro4, &temp4);
  float gyro4vec[3] = {gyro4.gyro.x, gyro4.gyro.y, gyro4.gyro.z};
  float omega4 = dotProduct(gyro4vec, mpuVec3);

  Serial.print(omega0); Serial.print("\t");
  Serial.println(omega4);

  /*Serial.print(gyro0.gyro.x); Serial.print("\t");
  Serial.print(gyro0.gyro.y); Serial.print("\t");
  Serial.print(gyro0.gyro.z); Serial.print("\t");
  Serial.print(gyro4.gyro.x); Serial.print("\t");
  Serial.print(gyro4.gyro.y); Serial.print("\t");
  Serial.println(gyro4.gyro.z);*/

  delay(50); // ~20 Hz update rate
}
