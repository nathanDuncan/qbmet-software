#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define ESP32_LIGHTPIN 2

// Create two MPU6050 objects
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

const int buttonPin = 25;
int buttonState = 0;
bool hasCalibrated = false;

// Calibration offsets
float xAccelOffset1, yAccelOffset1, zAccelOffset1;
float xGyroOffset1, yGyroOffset1, zGyroOffset1;
float xAccelOffset2, yAccelOffset2, zAccelOffset2;
float xGyroOffset2, yGyroOffset2, zGyroOffset2;

void calibrateSensors(Adafruit_MPU6050 &mpu, float &xAccelOffset, float &yAccelOffset, float &zAccelOffset,
                      float &xGyroOffset, float &yGyroOffset, float &zGyroOffset) {
  const int numSamples = 100;
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  Serial.println("Calibrating... Keep sensor stable!");

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    ax += a.acceleration.x;
    ay += a.acceleration.y;
    az += a.acceleration.z;
    gx += g.gyro.x;
    gy += g.gyro.y;
    gz += g.gyro.z;

    delay(10);
  }

  xAccelOffset = ax / numSamples;
  yAccelOffset = ay / numSamples;
  zAccelOffset = az / numSamples;
  xGyroOffset = gx / numSamples;
  yGyroOffset = gy / numSamples;
  zGyroOffset = gz / numSamples;

  Serial.println("Calibration complete.");
}

// Calculate pitch, yaw, roll from gyroscope data
void calculateAngles(float gx, float gy, float gz, float &pitch, float &yaw, float &roll) {
  pitch = gx * 180 / 3.14159;
  yaw = gy * 180 / 3.14159;
  roll = gz * 180 / 3.14159;
}

// Read sensor data and send over Serial
void readAndSendData(Adafruit_MPU6050 &mpu, float xAccelOffset, float yAccelOffset, float zAccelOffset,
                      float xGyroOffset, float yGyroOffset, float zGyroOffset, int sensorNum) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float pitch, yaw, roll;
  calculateAngles(g.gyro.x - xGyroOffset, g.gyro.y - yGyroOffset, g.gyro.z - zGyroOffset, pitch, yaw, roll);

  // Print data in CSV format
  Serial.print(sensorNum);
  Serial.print(",");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(a.acceleration.x - xAccelOffset);
  Serial.print(",");
  Serial.print(a.acceleration.y - yAccelOffset);
  Serial.print(",");
  Serial.print(a.acceleration.z - zAccelOffset);
  Serial.print(",");
  Serial.print(g.gyro.x - xGyroOffset);
  Serial.print(",");
  Serial.print(g.gyro.y - yGyroOffset);
  Serial.print(",");
  Serial.print(g.gyro.z - zGyroOffset);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(yaw);
  Serial.print(",");
  Serial.println(roll);
}

void setup() {
  Serial.begin(115200);
  delay(10000);
  while (!Serial) delay(10);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ESP32_LIGHTPIN, OUTPUT);

  if (!mpu1.begin(0x68)) {
    Serial.println("Failed to find MPU6050 at 0x68");
    while (1) delay(1000);
  }
  if (!mpu2.begin(0x69)) {
    Serial.println("Failed to find MPU6050 at 0x69");
    while (1) delay(1000);
  }

  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
}

void loop() {
  static bool collectingData = false;
  static unsigned long startTime = 0;

  if (digitalRead(buttonPin) == LOW) {
    if (!hasCalibrated) {
      Serial.println("Calibrating Sensor 1...");
      calibrateSensors(mpu1, xAccelOffset1, yAccelOffset1, zAccelOffset1, xGyroOffset1, yGyroOffset1, zGyroOffset1);
      Serial.println("Calibrating Sensor 2...");
      calibrateSensors(mpu2, xAccelOffset2, yAccelOffset2, zAccelOffset2, xGyroOffset2, yGyroOffset2, zGyroOffset2);
      hasCalibrated = true;
    } else {
      collectingData = true;
      startTime = millis();
      Serial.println("Sensor,Timestamp,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Pitch,Yaw,Roll");
    }
    delay(500);
  }

  if (collectingData) {
    if (millis() - startTime <= 5000) {
      readAndSendData(mpu1, xAccelOffset1, yAccelOffset1, zAccelOffset1, xGyroOffset1, yGyroOffset1, zGyroOffset1, 1);
      readAndSendData(mpu2, xAccelOffset2, yAccelOffset2, zAccelOffset2, xGyroOffset2, yGyroOffset2, zGyroOffset2, 2);
      delay(10);
    } else {
      collectingData = false;
      Serial.println("Data collection complete.");
    }
  }
}
