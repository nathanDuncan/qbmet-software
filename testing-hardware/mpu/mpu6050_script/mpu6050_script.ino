#include <Wire.h>
#include <MPU6050_tockn.h>

// Create the MPU6050 object
MPU6050 mpu6050(Wire);

// Specify which Pico pins for I2C (example: GP0 for SDA, GP1 for SCL)
const uint8_t SDA_PIN = 0; // GP0
const uint8_t SCL_PIN = 1; // GP1

// Variables to store current readings
float accX, accY, accZ;    // acceleration in g
float gyroX, gyroY, gyroZ; // rotation rate in °/s

// Variables to store previous readings for delta calculation
float oldAccX = 0.0, oldAccY = 0.0, oldAccZ = 0.0;
float oldGyroX = 0.0, oldGyroY = 0.0, oldGyroZ = 0.0;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting MPU6050...");

  // Set which pins to use for I2C (optional if already default)
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);

  // Begin the I2C bus
  Wire.begin();

  // Initialize MPU6050
  mpu6050.begin();

  // (Optional) Calibrate gyro offsets. The 'true' argument prints offsets to Serial.
  // Keep the sensor still while calibrating!
  mpu6050.calcGyroOffsets(true);

  Serial.println("MPU6050 Initialized.");
}

void loop() {
  // Update the MPU6050 readings (must be called each loop)
  mpu6050.update();

  // Current accelerations in g
  accX = mpu6050.getAccX();
  accY = mpu6050.getAccY();
  accZ = mpu6050.getAccZ();

  // Current gyro rates in °/s
  gyroX = mpu6050.getGyroX();
  gyroY = mpu6050.getGyroY();
  gyroZ = mpu6050.getGyroZ();

  // Calculate deltas (difference from previous reading)
  float deltaAccX  = accX  - oldAccX;
  float deltaAccY  = accY  - oldAccY;
  float deltaAccZ  = accZ  - oldAccZ;
  float deltaGyroX = gyroX - oldGyroX;
  float deltaGyroY = gyroY - oldGyroY;
  float deltaGyroZ = gyroZ - oldGyroZ;

  // Print current readings
  Serial.println("=== Current Readings ===");
  Serial.print("AccX: ");  Serial.print(accX, 3);   Serial.print(" g, ");
  Serial.print("AccY: ");  Serial.print(accY, 3);   Serial.print(" g, ");
  Serial.print("AccZ: ");  Serial.print(accZ, 3);   Serial.println(" g");

  Serial.print("GyroX: "); Serial.print(gyroX, 3);  Serial.print(" °/s, ");
  Serial.print("GyroY: "); Serial.print(gyroY, 3);  Serial.print(" °/s, ");
  Serial.print("GyroZ: "); Serial.print(gyroZ, 3);  Serial.println(" °/s");

  // Print deltas
  Serial.println("=== Delta Readings ===");
  Serial.print("ΔAccX: ");  Serial.print(deltaAccX, 3);   Serial.print(" g, ");
  Serial.print("ΔAccY: ");  Serial.print(deltaAccY, 3);   Serial.print(" g, ");
  Serial.print("ΔAccZ: ");  Serial.print(deltaAccZ, 3);   Serial.println(" g");

  Serial.print("ΔGyroX: "); Serial.print(deltaGyroX, 3);  Serial.print(" °/s, ");
  Serial.print("ΔGyroY: "); Serial.print(deltaGyroY, 3);  Serial.print(" °/s, ");
  Serial.print("ΔGyroZ: "); Serial.print(deltaGyroZ, 3);  Serial.println(" °/s");

  Serial.println();

  // Store current readings to old values for next loop
  oldAccX  = accX;
  oldAccY  = accY;
  oldAccZ  = accZ;
  oldGyroX = gyroX;
  oldGyroY = gyroY;
  oldGyroZ = gyroZ;

  // Delay for readability (adjust as needed)
  delay(200);
}
