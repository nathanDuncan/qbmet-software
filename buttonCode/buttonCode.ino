// Define the button pin
const int buttonPin = 18;  // Change to the appropriate GPIO pin
const int buttonPin2 = 19;
int buttonState = 0;       // Variable to hold the button state
int buttonState2 = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set the button pin as input
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
}

void loop() {
  // Read the state of the button
  buttonState = digitalRead(buttonPin);
  buttonState2 = digitalRead(buttonPin2);

  // Check if the button is pressed
  if (buttonState == HIGH) {
    // Print "High" to the serial monitor
    Serial.println("Forward");
  }

  if (buttonState2 == HIGH){
    Serial.println("Back");
  }
  //else if (buttonState == LOW) {
    // Print "Low" to the serial monitor
    //Serial.println("Low");
  //}

  // Wait for a short period to avoid multiple prints for a single press
  delay(500);
}
