#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Joystick Pins
#define JOYSTICK_Y A1 // Y-axis is used for forward and backward movement

// Push Buttons 
#define BUTTON_START 2  // Start Robot
#define BUTTON_STOP 3   // Stop Robot
#define BUTTON_PID 4    // PID Mode
#define BUTTON_POLE 5   // Pole Placement Mode

// nRF24L01 Pins 
#define CE_PIN 7
#define CSN_PIN 8

// RF24 set as object
RF24 radio(CE_PIN, CSN_PIN);

// communication address
const byte address[6] = "00001";

// Data structure for sending controls
struct ControlData {
  int forward;   // Forward speed
  int backward;  // Backward speed
  int spin;      // Spin control
  int startStop; // Start/Stop state (1 = Start, 0 = Stop)
};

ControlData controlData;

void setup() {
  // Initialize joystick pin
  pinMode(JOYSTICK_Y, INPUT);

  // Initialize buttons
  pinMode(BUTTON_START, INPUT_PULLUP);
  pinMode(BUTTON_STOP, INPUT_PULLUP);
  pinMode(BUTTON_PID, INPUT_PULLUP);
  pinMode(BUTTON_POLE, INPUT_PULLUP);

  // Initialize serial monitor 
  Serial.begin(9600);

  // Initialize nRF24L01
  radio.begin();
  radio.openWritingPipe(address);  // Set as transmitter with address
  radio.setPALevel(RF24_PA_LOW);   // Low power mode
  radio.stopListening();           // Set to sending mode
}

void loop() {
  // Read joystick value for forward/backward movement
  int yValue = analogRead(JOYSTICK_Y);

  // Add a dead zone to filter noise near the center
  int deadZone = 20;

  if (yValue > (512 + deadZone)) {
    controlData.forward = map(yValue, 512 + deadZone, 1023, 0, 700);
    controlData.backward = 0;
  } else if (yValue < (512 - deadZone)) {
    controlData.backward = map(yValue, 512 - deadZone, 0, 0, -700);
    controlData.forward = 0;
  } else {
    controlData.forward = 0;
    controlData.backward = 0;
  }

  // Check spin control
 // controlData.spin = digitalRead(BUTTON_POLE) == LOW ? 1 : (digitalRead(BUTTON_PID) == LOW ? -1 : 0); //void it

  // Check start/stop buttons
  if (digitalRead(BUTTON_START) == LOW) {
    controlData.startStop = 1; // Start robot
  } else if (digitalRead(BUTTON_STOP) == LOW) {
    controlData.startStop = 0; // Stop robot
  }

  // Send data car
  bool success = radio.write(&controlData, sizeof(controlData));

  // Debugging output 
  Serial.print("Sending data... ");
  Serial.println(success ? "Success" : "Failed");

  Serial.print("Forward: ");
  Serial.println(controlData.forward)
  Serial.print("Backward: ");
  Serial.println(controlData.backward);
  Serial.print("Spin: ");
  Serial.println(controlData.spin);
  Serial.print("Start/Stop: ");
  Serial.println(controlData.startStop);

  delay(50); // Avoid flooding the communication channel
}
