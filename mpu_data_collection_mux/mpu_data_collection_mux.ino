#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// 0 is motor1 (thigh)
// 1 is motor2 (calf)
// 4 is motor3 (thigh)
// 5 is motor4 (calf)

// Pin definitions
#define SDA_PIN 23
#define SCL_PIN 25
#define PCA9548A_ADDR 0x70
#define PCA_A0_PIN 32
#define PCA_A1_PIN 33

// Select multiplexer channel (0-7)
void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

Adafruit_MPU6050 mpu;

const unsigned long DURATION_MS = 3000; // 5 seconds
const unsigned long SAMPLE_INTERVAL_MS = 10; // 100Hz (adjust as needed)

void setup() {
  Serial.begin(115200);
  Wire.begin(23, 25); // Use your actual I2C pins (or 21,22 for default)
  delay(1000);

  // If using PCA9548A, select channel here:
  selectMuxChannel(0);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  Serial.println("time_ms,gx,gy,gz,ax,ay,az"); // CSV header
  delay(1000); // Give you time to open Serial Monitor
}

void loop() {
  static bool collected = false;
  if (!collected) {
    unsigned long start = millis();
    while (millis() - start < DURATION_MS) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      unsigned long t = millis() - start;
      Serial.print(t);
      Serial.print(",");
      Serial.print(g.gyro.x, 6); Serial.print(",");
      Serial.print(g.gyro.y, 6); Serial.print(",");
      Serial.print(g.gyro.z, 6); Serial.print(",");
      Serial.print(a.acceleration.x, 6); Serial.print(",");
      Serial.print(a.acceleration.y, 6); Serial.print(",");
      Serial.println(a.acceleration.z, 6);
      delay(SAMPLE_INTERVAL_MS);
    }
    collected = true;
    Serial.println("DATA COLLECTION COMPLETE");
    // Optionally stop here:
    while (1);
  }
}

