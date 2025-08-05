#include <TMCStepper.h>
#include <SoftwareSerial.h>

// UART pins
#define SW_RX 5           // Unused, but required for SoftwareSerial
#define SW_TX 4           // Connect this to TMC2209 UART (via 1kΩ resistor)
#define R_SENSE 0.15f     // Adjust this based on your driver board
#define DRIVER_ADDR 0b00  // Default slave address for TMC2209

// Motor control pins
#define EN_PIN 8
#define DIR_PIN 3
#define STEP_PIN 2

SoftwareSerial SoftSerial(SW_RX, SW_TX);  // RX, TX
TMC2209Stepper driver(&SoftSerial, R_SENSE, DRIVER_ADDR);

unsigned long lastReceiveTime = 0;
const unsigned long timeoutThreshold = 5000;

void setup() {
  // Setup control pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);    // Enable driver (active LOW)
  digitalWrite(DIR_PIN, HIGH);  // Set initial direction

  // Start serial connections
  // Serial.begin(115200);        // For debugging
  SoftSerial.begin(115200);  // UART to TMC2209

  delay(500);  // Give time for power to stabilize

  // Initialize driver via UART
  driver.begin();
  driver.toff(5);                // Enables driver (must be >0!)
  driver.rms_current(800);       // Set motor RMS current (adjust to your motor)
  driver.microsteps(16);         // 16 microsteps = smooth and common
  driver.en_spreadCycle(false);  // Enable StealthChop
  driver.pwm_autoscale(true);    // Enable autoscaling of PWM
  driver.pwm_autograd(true);     // Optional: smoother transitions
  driver.TCOOLTHRS(0xFFFFF);     // Allow full speed in StealthChop
  driver.semin(5);               // Coolstep minimum
  driver.semax(2);               // Coolstep maximum
  driver.shaft(true);            // Set direction (true = forward)

  Serial.println("TMC2209 initialized in UART mode.");

  Serial.begin(1000000);

  lastReceiveTime = millis();
}

float currentAngle = 0;  // start at 0 degrees
int stepCount = 0;
int angleFromStart = 0;

void loop() {
  if (Serial.available()) {
    float error = Serial.parseFloat() * 4.44;
    lastReceiveTime = millis();

    if (error > 5) {
      // turn right
      digitalWrite(DIR_PIN, HIGH);
      while (error > 5) {
        for (int i = 0; i < 3; i++) {
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(1000);
          digitalWrite(STEP_PIN, LOW);
          delayMicroseconds(1000);
          stepCount++;
        }
        if (Serial.available()) {
          error = Serial.parseFloat() * 4.44;
        }
      }
    }

    if (error < -5) {
      // turn left
      digitalWrite(DIR_PIN, LOW);
      while (error < -5) {
        for (int i = 0; i < 3; i++) {
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(1000);
          digitalWrite(STEP_PIN, LOW);
          delayMicroseconds(1000);
          stepCount--;
        }
        if (Serial.available()) {
          error = Serial.parseFloat() * 4.44;
        }
      }
    }



    delay(10);
  }
  if (millis() - lastReceiveTime >= timeoutThreshold) {
    if (stepCount > 0) {
      digitalWrite(DIR_PIN, LOW);
      angleFromStart = abs(stepCount * 4.44);
      for (int i = 0; i < abs(stepCount); i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(1000);
      }
      stepCount = 0;
    } else if (stepCount < 0) {
      digitalWrite(DIR_PIN, HIGH);
      angleFromStart = abs(stepCount * 4.44);
      for (int i = 0; i < abs(stepCount); i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(1000);
      }
      stepCount = 0;
    }
  }
}
