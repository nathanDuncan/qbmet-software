#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <math.h>

// ----- Wi‑Fi and TCP Server Settings -----
const char* ssid     = "396 Johnson";
const char* password = "#396J/king0nt";
const char* serverIP = "192.168.5.40";  // Your Mac's IP address running the TCP server
const uint16_t serverPort = 1234;       // TCP server port

// ----- Button and Sampling Settings -----
const int buttonPin = 25;         // Push button on GPIO25 (active LOW)
#define SAMPLE_INTERVAL 10        // Sample every 10 ms
#define COLLECTION_DURATION 10000 // Collect for 10000 ms (10 seconds)

// ----- Create Two MPU6050 Objects -----
Adafruit_MPU6050 mpu1;  // Sensor 1 at address 0x68 (AD0 tied to GND)
Adafruit_MPU6050 mpu2;  // Sensor 2 at address 0x69 (AD0 tied to VCC)

// ----- WiFi Client -----
WiFiClient client;

// For yaw integration (in degrees) for each sensor
float yaw1 = 0.0, yaw2 = 0.0;
unsigned long lastIntegrationTime = 0;

// Dataset counter
int datasetCount = 0;

// Calibration offsets for both sensors
struct SensorOffsets {
  float accel[3];
  float gyro[3];
};
SensorOffsets offsets1, offsets2;
bool hasCalibrated = false;

// ----- Calibration Function -----
void calibrateSensor(Adafruit_MPU6050 &mpu, SensorOffsets &offsets) {
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

  offsets.accel[0] = ax / numSamples;
  offsets.accel[1] = ay / numSamples;
  offsets.accel[2] = az / numSamples;
  offsets.gyro[0] = gx / numSamples;
  offsets.gyro[1] = gy / numSamples;
  offsets.gyro[2] = gz / numSamples;

  Serial.println("Calibration complete.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for Serial to be ready
  }

  Wire.begin();
  pinMode(buttonPin, INPUT_PULLUP);

  // ----- Connect to Wi‑Fi -----
  Serial.print("Connecting to Wi‑Fi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Wi‑Fi connected. ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // ----- Initialize Sensor 1 -----
  if (!mpu1.begin(0x68)) {
    Serial.println("Failed to find MPU6050 sensor 1 (0x68)");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Sensor 1 initialized.");

  // ----- Initialize Sensor 2 -----
  if (!mpu2.begin(0x69)) {
    Serial.println("Failed to find MPU6050 sensor 2 (0x69)");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Sensor 2 initialized.");

  pinMode(2, OUTPUT);

  // Set sensor ranges and filter bandwidth for both sensors
  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);

  mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu2.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize integration timer
  lastIntegrationTime = millis();

  Serial.println("Ready. Press the button to start a 10-second data collection.");
}

void loop() {
  // Wait for a button press (active LOW)
  if (digitalRead(buttonPin) == LOW) {
    unsigned long buttonPressTime = millis();

    // Check if the button is held for more than 3 seconds for calibration
    while (digitalRead(buttonPin) == LOW) {
      if (millis() - buttonPressTime > 3000) {
        Serial.println("Calibrating Sensor 1...");
        calibrateSensor(mpu1, offsets1);
        Serial.println("Calibrating Sensor 2...");
        calibrateSensor(mpu2, offsets2);
        hasCalibrated = true;
        Serial.println("Calibration complete.");
        return;
      }
    }

    // If calibration is done, start data collection
    if (hasCalibrated) {
      datasetCount++;  // New dataset
      Serial.print("Button pressed: starting dataset ");
      Serial.println(datasetCount);

      // Attempt to connect to the TCP server
      if (!client.connect(serverIP, serverPort)) {
        Serial.println("Failed to connect to TCP server.");
        return;
      }

      // Send a unique dataset header
      char datasetHeader[64];
      sprintf(datasetHeader, "----- Dataset: Dataset_%03d -----", datasetCount);
      client.println(datasetHeader);
      Serial.println(datasetHeader);

      // Send CSV column header
      client.println("Time_ms,Pitch1,Yaw1,Roll1,RawAx1,RawAy1,RawAz1,RawGx1,RawGy1,RawGz1,Pitch2,Yaw2,Roll2,RawAx2,RawAy2,RawAz2,RawGx2,RawGy2,RawGz2");
      Serial.println("Time_ms,Pitch1,Yaw1,Roll1,RawAx1,RawAy1,RawAz1,RawGx1,RawGy1,RawGz1,Pitch2,Yaw2,Roll2,RawAx2,RawAy2,RawAz2,RawGx2,RawGy2,RawGz2");

      // Reset yaw integration for each sensor
      yaw1 = 0.0;
      yaw2 = 0.0;
      unsigned long collectionStart = millis();
      lastIntegrationTime = collectionStart;
      unsigned long previousSampleTime = collectionStart;

      // Collect data for exactly 10 seconds (regardless of button state)
      while (millis() - collectionStart < COLLECTION_DURATION) {
        digitalWrite(2, HIGH);
        unsigned long currentTime = millis();
        if (currentTime - previousSampleTime < SAMPLE_INTERVAL)
          continue;  // Wait until 10 ms have passed

        float dt = (currentTime - lastIntegrationTime) / 1000.0; // delta time in seconds
        lastIntegrationTime = currentTime;
        previousSampleTime = currentTime;

        // ----- Read sensor events from both sensors -----
        sensors_event_t a1, g1, temp1;
        sensors_event_t a2, g2, temp2;
        mpu1.getEvent(&a1, &g1, &temp1);
        mpu2.getEvent(&a2, &g2, &temp2);

        // Apply calibration offsets
        a1.acceleration.x -= offsets1.accel[0];
        a1.acceleration.y -= offsets1.accel[1];
        a1.acceleration.z -= offsets1.accel[2];
        g1.gyro.x -= offsets1.gyro[0];
        g1.gyro.y -= offsets1.gyro[1];
        g1.gyro.z -= offsets1.gyro[2];

        a2.acceleration.x -= offsets2.accel[0];
        a2.acceleration.y -= offsets2.accel[1];
        a2.acceleration.z -= offsets2.accel[2];
        g2.gyro.x -= offsets2.gyro[0];
        g2.gyro.y -= offsets2.gyro[1];
        g2.gyro.z -= offsets2.gyro[2];

        // --- Sensor 1 Data ---
        float raw_ax1 = a1.acceleration.x;
        float raw_ay1 = a1.acceleration.y;
        float raw_az1 = a1.acceleration.z;
        float raw_gx1 = g1.gyro.x;
        float raw_gy1 = g1.gyro.y;
        float raw_gz1 = g1.gyro.z;

        // Remap accelerometer data for sensor 1 (rotate +90° about X):
        float ax1 = raw_ax1;
        float ay1 = -raw_az1;
        float az1 = raw_ay1;
        float az_corr1 = -az1;
        float pitch1 = atan2(-ax1, sqrt(ay1 * ay1 + az_corr1 * az_corr1)) * 180.0 / PI;
        float roll1  = atan2(ay1, az_corr1) * 180.0 / PI;

        // Remap gyro data for sensor 1 (rotate +90° about X):
        float gx1 = raw_gx1;
        float gy1 = -raw_gz1;
        float gz1 = raw_gy1;
        yaw1 += gz1 * dt * 180.0 / PI;

        // --- Sensor 2 Data ---
        float raw_ax2 = a2.acceleration.x;
        float raw_ay2 = a2.acceleration.y;
        float raw_az2 = a2.acceleration.z;
        float raw_gx2 = g2.gyro.x;
        float raw_gy2 = g2.gyro.y;
        float raw_gz2 = g2.gyro.z;

        float ax2 = raw_ax2;
        float ay2 = -raw_az2;
        float az2 = raw_ay2;
        float az_corr2 = -az2;
        float pitch2 = atan2(-ax2, sqrt(ay2 * ay2 + az_corr2 * az_corr2)) * 180.0 / PI;
        float roll2  = atan2(ay2, az_corr2) * 180.0 / PI;

        float gx2 = raw_gx2;
        float gy2 = -raw_gz2;
        float gz2 = raw_gy2;
        yaw2 += gz2 * dt * 180.0 / PI;

        // Create a CSV line with elapsed time and data from both sensors
        unsigned long elapsed = currentTime - collectionStart;
        char csvLine[300];
        sprintf(csvLine,
                "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                elapsed,
                pitch1, yaw1, roll1,
                raw_ax1, raw_ay1, raw_az1,
                raw_gx1, raw_gy1, raw_gz1,
                pitch2, yaw2, roll2,
                raw_ax2, raw_ay2, raw_az2,
                raw_gx2, raw_gy2, raw_gz2);

        // Send the CSV line over TCP and also print to Serial
        client.println(csvLine);
        Serial.println(csvLine);
      } // End of 10-second data collection loop
      digitalWrite(2, LOW);
      Serial.println("Dataset collection complete.");
      client.stop();
    } else {
      Serial.println("Please calibrate sensors before collecting data.");
    }
  } else {
    delay(10);
  }
}