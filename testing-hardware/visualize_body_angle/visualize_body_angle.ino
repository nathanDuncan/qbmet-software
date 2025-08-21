#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- CONFIGURATION ---
#define SDA_PIN 23
#define SCL_PIN 25
#define PCA9548A_ADDR 0x70  // Multiplexer I2C address

#define MUX_CHANNEL 0  // Change this to select the correct MPU channel

Adafruit_MPU6050 mpu;

// Gravity vector when standing upright (specific to this MPU orientation)
float gravity_resting[3] = {-0.981, 0.00, 0.00};

// Filtered gravity vector
float gravity_filtered[3] = {0, 0, 0};
const float alpha = 0.9;

// --- Helper Functions ---
float vectorNorm(const float v[3]) {
  return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void normalize(float v[3]) {
  float norm = vectorNorm(v);
  if (norm == 0) return;
  for (int i = 0; i < 3; i++) v[i] /= norm;
}

float dotProduct(const float a[3], const float b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void lowPassFilter(const float input[3], float output[3]) {
  for (int i = 0; i < 3; i++) {
    output[i] = alpha * output[i] + (1 - alpha) * input[i];
  }
}

// --- Multiplexer Channel Selector ---
void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);

  // Select MPU via multiplexer
  selectMuxChannel(MUX_CHANNEL);
  delay(100);

  if (!mpu.begin()) {
    Serial.println("MPU not found on selected mux channel!");
    while (1);
  }

  // Initialize gravity filter with first reading
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  gravity_filtered[0] = accel.acceleration.x;
  gravity_filtered[1] = accel.acceleration.y;
  gravity_filtered[2] = accel.acceleration.z;

  Serial.println("angle_deg");
}

// --- Loop ---
void loop() {
  // Select correct MPU channel each loop (in case others are used elsewhere)
  selectMuxChannel(MUX_CHANNEL);
  delayMicroseconds(500);  // Short delay for mux switching stability

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float accel_raw[3] = {
    accel.acceleration.x,
    accel.acceleration.y,
    accel.acceleration.z
  };
  lowPassFilter(accel_raw, gravity_filtered);

  float g1[3] = {gravity_resting[0], gravity_resting[1], gravity_resting[2]};
  float g2[3] = {gravity_filtered[0], gravity_filtered[1], gravity_filtered[2]};
  normalize(g1);
  normalize(g2);

  float cos_theta = dotProduct(g1, g2);
  cos_theta = constrain(cos_theta, -1.0, 1.0);
  float angle_rad = acos(cos_theta);
  float angle_deg = angle_rad * 180.0 / PI;

  Serial.println(angle_deg);

  delay(10);  // 100 Hz
}
