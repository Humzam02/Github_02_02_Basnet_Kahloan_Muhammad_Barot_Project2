#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <MPU6050.h>
#include "KalmanFilter.h"

// RF24 Pins
#define CE_PIN 5
#define CSN_PIN 3

RF24 radio(CE_PIN, CSN_PIN);

// Communication address
const byte address[6] = "00001";

struct ControlData {
    int forward;   // Forward speed
    int backward;  // Backward speed
    int startStop; // PB2 state (1 = Go, 0 = Stop)
    int stopBot;   // PB1 state (1 = Stop, 0 = Continue)
};
ControlData controlData;

// Motor Pins
#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8

// Pole Placement Button Pin
#define PB3_PIN 4

// Global variables 
bool isStarted = false;        // Tracks whether the bot is allowed to move
bool joystickActive = false;   // Tracks whether joystick input is active
bool botStopped = false;       // Tracks whether PB1 was pressed
bool polePlacementMode = false;// Tracks whether pole placement mode is active
MPU6050 mpu;
KalmanFilter kalman;

// PID parameters
double kp = 55.0, ki = 0.0, kd = 0.75; // PID gains
double angleSetpoint = 0.0;            // Desired upright angle (0 degrees tilt)
double currentAngle = 0.0, error = 0.0, previousError = 0.0;
double integral = 0.0, derivative = 0.0;
double dt = 0.005; // 5ms loop interval

// Pole Placement Parameters
const double K[4] = {1.5, 2.8, -1.2, -0.8}; // gain matrix

// State variables
double state[4] = {0, 0, 0, 0}; // x1, x2, x3, x4

// Motor speed variables
int motorSpeedLeft = 0;
int motorSpeedRight = 0;

void setup() {
    // Motor Pins
    pinMode(IN1M, OUTPUT);
    pinMode(IN2M, OUTPUT);
    pinMode(IN3M, OUTPUT);
    pinMode(IN4M, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);

    // PB3 Button Pin
    pinMode(PB3_PIN, INPUT_PULLUP);

    // Initialize Serial and I2C
    Wire.begin();
    Serial.begin(9600);

    // Initialize Radio
    radio.begin();
    radio.openReadingPipe(1, address);  // Set as receiver with address
    radio.setPALevel(RF24_PA_LOW);      // Low power mode
    radio.startListening();             // Set to listening mode

    // Initialize Motors
    digitalWrite(STBY, LOW); // Ensure motors start off
    Serial.println("Bot is ready and waiting for the GO button...");

    // Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    if (mpu.testConnection()) {
        Serial.println("MPU6050 connected successfully!");
    } else {
        Serial.println("MPU6050 connection failed!");
        while (true); // STOP ALL if MPU6050 is off
    }
}

void loop() {
    // Read PB3 button state for pole placement
    if (digitalRead(PB3_PIN) == LOW) { // Button pressed (LOW because of INPUT_PULLUP)
        if (!polePlacementMode) {
            Serial.println("PB3 pressed! Switching to Pole Placement Control...");
            polePlacementMode = true;
        }
        Serial.println("Stabilizing through Pole Placement Control...");
        polePlacementControl();
        return; // Pole Control only
    } else {
        if (polePlacementMode) {
            Serial.println("Exiting Pole Placement Control...");
        }
        polePlacementMode = false;
    }

    if (radio.available()) {
        radio.read(&controlData, sizeof(controlData));

        // Handle PB1 (Stop Bot)
        if (controlData.stopBot == 1) {
            botStopped = true;
            Serial.println("PB1 pressed! Bot stopped.");
            stopMotors();
            return;
        } else {
            botStopped = false; // continue normal operation
        }

        // Check for start signal (PB2)
        if (controlData.startStop == 1 && !isStarted) {
            isStarted = true; // Allow the bot to move
            Serial.println("Start signal received! Stabilizing through PID Control...");
        }

        // If not started, ignore all motor commands and stabilization
        if (!isStarted) {
            Serial.println("Bot is waiting for the GO button...");
            return;
        }

        //Joystick Logic
        if (controlData.forward > 500 || controlData.backward < -500) {
            joystickActive = true; // Activate joystick control

            if (controlData.forward > 500) {
                Serial.println("Joystick input detected: Moving Forward...");
                moveForward();
            } else if (controlData.backward < -500) {
                Serial.println("Joystick input detected: Moving Backward...");
                moveBackward();
            }
            return; // NO PID control while joystick is active
        } else {
            joystickActive = false; // No joystick input; back to to PID control
        }

        // Self-Stabilization (PID)
        if (!joystickActive) {
            Serial.println("Stabilizing through PID Control...");
            pidControl();
        }
    }
}

// Pole Placement Control Logic
void polePlacementControl() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    state[0] = currentAngle;
    state[1] = gx / 131.0;
    state[2] = ax / 16384.0;
    state[3] = ay / 16384.0;

    double controlInput = 0;
    for (int i = 0; i < 4; i++) {
        controlInput -= K[i] * state[i];
    }

    applyMotorControl(controlInput);
}

// PID Control Logic
void pidControl() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    kalman.Angle(ax, ay, az, gx, gy, gz, dt, 0.001, 0.005, 0.5, 1, 0.05);
    currentAngle = kalman.angle;

    error = angleSetpoint - currentAngle;
    integral += error * dt;
    derivative = (error - previousError) / dt;
    double pidOutput = (kp * error) + (ki * integral) + (kd * derivative);
    previousError = error;

    applyMotorControl(pidOutput);
}

// Function motor control
void applyMotorControl(double controlInput) {
    motorSpeedLeft = motorSpeedRight = constrain(controlInput, -255, 255);

    if (motorSpeedLeft > 0) {
        digitalWrite(IN1M, HIGH);
        digitalWrite(IN2M, LOW);
        analogWrite(PWMA, motorSpeedLeft);
    } else {
        digitalWrite(IN1M, LOW);
        digitalWrite(IN2M, HIGH);
        analogWrite(PWMA, -motorSpeedLeft);
    }

    if (motorSpeedRight > 0) {
        digitalWrite(IN3M, HIGH);
        digitalWrite(IN4M, LOW);
        analogWrite(PWMB, motorSpeedRight);
    } else {
        digitalWrite(IN3M, LOW);
        digitalWrite(IN4M, HIGH);
        analogWrite(PWMB, -motorSpeedRight);
    }
}

// Move Forward
void moveForward() {
    digitalWrite(STBY, HIGH);
    analogWrite(PWMA, 255);
    analogWrite(PWMB, 255);
    digitalWrite(IN1M, LOW);
    digitalWrite(IN2M, HIGH);
    digitalWrite(IN3M, HIGH);
    digitalWrite(IN4M, LOW);
}

// Move Backward
void moveBackward() {
    digitalWrite(STBY, HIGH);
    analogWrite(PWMA, 255);
    analogWrite(PWMB, 255);
    digitalWrite(IN1M, HIGH);
    digitalWrite(IN2M, LOW);
    digitalWrite(IN3M, LOW);
    digitalWrite(IN4M, HIGH);
}

// Stop Motors
void stopMotors() {
    digitalWrite(STBY, LOW);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(IN1M, LOW);
    digitalWrite(IN2M, LOW);
    digitalWrite(IN3M, LOW);
    digitalWrite(IN4M, LOW);
}
