#include <ACAN2515.h>

// Define SPI pins for MCP2515
static const byte MCP2515_SCK  = 4 ; // SCK  (GP18)
static const byte MCP2515_MOSI = 6 ; // MOSI (GP19)
static const byte MCP2515_MISO = 5 ; // MISO (GP16)
static const byte MCP2515_CS   = 2 ; // CS   (GP17)
static const byte MCP2515_INT  = 20 ; // INT  (GP20)

// Built-in LED on Pico W (GP25)
const int LED_PIN = 25;

// Initialize MCP2515 object
ACAN2515 can(MCP2515_CS, SPI, MCP2515_INT);

// CAN Settings (MCP2515 crystal frequency + desired baud rate)
static const uint32_t MCP2515_QUARTZ_FREQUENCY = 16UL * 1000UL * 1000UL; // 16 MHz crystal
static const uint32_t CAN_BAUD_RATE = 500UL * 1000UL ; // 500 kbps

void setup() {
  pinMode(LED_PIN, OUTPUT); // Initialize LED
  Serial.begin(115200);
  Serial.println("Setup started"); // This will print immediately

  SPI.setSCK(MCP2515_SCK);
  SPI.setTX(MCP2515_MOSI);
  SPI.setRX(MCP2515_MISO);
  SPI.begin();

  ACAN2515Settings settings(MCP2515_QUARTZ_FREQUENCY, CAN_BAUD_RATE);
  settings.mRequestedMode = ACAN2515Settings::NormalMode;

  const uint16_t errorCode = can.begin(settings, [] { can.isr(); });

  if (errorCode == 0) {
    Serial.println("CAN initialized!");
    digitalWrite(LED_PIN, HIGH); // LED ON if initialized
  } else {
    Serial.print("CAN error: 0x");
    Serial.println(errorCode, HEX);
    // Blink LED forever if initialization fails
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
}

void loop() {
  CANMessage message;
  message.id = 0x123;
  message.len = 8;
  message.data[0] = 0x01;
  message.data[1] = 0x02;
  message.data[2] = 0x03;
  message.data[3] = 0x04;
  message.data[4] = 0x05;
  message.data[5] = 0x06;
  message.data[6] = 0x07;
  message.data[7] = 0x08;

  if (can.tryToSend(message)) {
    Serial.println("Message sent");
    digitalWrite(LED_PIN, HIGH); // LED stays ON
    delay(1000);
  } else {
    Serial.println("Send failed");
    // Blink LED rapidly for 1 second on failure
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED
      delay(100); // Adjust delay to change blink speed
    }
  }
}