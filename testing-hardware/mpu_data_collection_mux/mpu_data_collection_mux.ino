#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// mux ID 0 is motor1 (thigh)
// mux ID 1 is motor2 (calf)
// mux ID 4 is motor3 (thigh)
// mux ID 5 is motor4 (calf)

// Pin definitions
#define SDA_PIN 23
#define SCL_PIN 25
#define PCA9548A_ADDR 0x70
#define PCA_A0_PIN 32
#define PCA_A1_PIN 33

// Select multiplexer channel
void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

Adafruit_MPU6050 mpu;

const unsigned long DURATION_MS = 3000; // 3 seconds of data
const unsigned long SAMPLE_INTERVAL_MS = 10; // 100Hz

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(5000);

  selectMuxChannel(4);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  // Prompt user to press Enter
  Serial.println("====================================");
  Serial.println(" PRESS ENTER TO BEGIN DATA COLLECTION ");
  Serial.println("====================================");

  // Wait for input from Serial Monitor
  while (!Serial.available()) {
    delay(100);
  }
  while (Serial.available()) {
    Serial.read(); // Clear the input buffer
  }

  Serial.println("========== BEGIN DATA COLLECTION ==========");
  Serial.println("time_ms,gx,gy,gz,ax,ay,az"); // CSV header
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
    Serial.println("========== DATA COLLECTION COMPLETE ==========");
    while (1); // halt
  }
}
