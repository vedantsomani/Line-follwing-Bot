#include <AFMotor.h>

// Motor Pins (L298N Motor Driver)
#define MOTOR_LEFT_FRONT_FORWARD  4
#define MOTOR_LEFT_FRONT_BACKWARD 5
#define MOTOR_RIGHT_FRONT_FORWARD 6
#define MOTOR_RIGHT_FRONT_BACKWARD 7
#define MOTOR_LEFT_BACK_FORWARD 8
#define MOTOR_LEFT_BACK_BACKWARD 9
#define MOTOR_RIGHT_BACK_FORWARD 10
#define MOTOR_RIGHT_BACK_BACKWARD 11

// IR Sensor Pins
#define IR_LEFT_FRONT A0
#define IR_RIGHT_FRONT A1
#define IR_LEFT_BACK A2
#define IR_RIGHT_BACK A3

// LED Pins
#define LED_LEFT_FRONT 12
#define LED_RIGHT_FRONT 13
#define LED_LEFT_BACK 14
#define LED_RIGHT_BACK 15

// Ultrasonic Sensor Pins
#define TRIG_PIN 18
#define ECHO_PIN 19

// Bluetooth Control Pins
char command; // Stores Bluetooth command

// PID Constants
float Kp = 1.0;  // Proportional constant
float Ki = 0.0;  // Integral constant
float Kd = 0.0;  // Derivative constant

// PID variables
float previous_error = 0;
float integral = 0;
float error = 0;
float output = 0;

// Motor objects
AF_DCMotor motor_left_front(1);
AF_DCMotor motor_right_front(2);
AF_DCMotor motor_left_back(3);
AF_DCMotor motor_right_back(4);

// Setup
void setup() {
  // Set the motor pins as outputs
  pinMode(IR_LEFT_FRONT, INPUT);
  pinMode(IR_RIGHT_FRONT, INPUT);
  pinMode(IR_LEFT_BACK, INPUT);
  pinMode(IR_RIGHT_BACK, INPUT);
  pinMode(LED_LEFT_FRONT, OUTPUT);
  pinMode(LED_RIGHT_FRONT, OUTPUT);
  pinMode(LED_LEFT_BACK, OUTPUT);
  pinMode(LED_RIGHT_BACK, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.begin(9600); // Initialize Bluetooth communication

  // Set motor speed
  motor_left_front.setSpeed(255);
  motor_right_front.setSpeed(255);
  motor_left_back.setSpeed(255);
  motor_right_back.setSpeed(255);
  
  digitalWrite(TRIG_PIN, LOW);
  delay(1000);
}

// Loop
void loop() {
  // Read IR sensor values
  int leftFront = analogRead(IR_LEFT_FRONT);
  int rightFront = analogRead(IR_RIGHT_FRONT);
  int leftBack = analogRead(IR_LEFT_BACK);
  int rightBack = analogRead(IR_RIGHT_BACK);

  // Turn on LEDs based on IR sensor readings
  digitalWrite(LED_LEFT_FRONT, leftFront < 500 ? HIGH : LOW);
  digitalWrite(LED_RIGHT_FRONT, rightFront < 500 ? HIGH : LOW);
  digitalWrite(LED_LEFT_BACK, leftBack < 500 ? HIGH : LOW);
  digitalWrite(LED_RIGHT_BACK, rightBack < 500 ? HIGH : LOW);

  // Calculate PID error
  error = leftFront - rightFront;  // Error between the left and right front sensors
  integral += error;
  float derivative = error - previous_error;
  output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;

  // Move the robot based on PID output
  if (output > 0) {
    moveLeft();
  } else if (output < 0) {
    moveRight();
  } else {
    moveForward();
  }

  // Check for obstacles with the ultrasonic sensor
  long distance = getDistance();
  if (distance < 20) {
    stopMotors(); // Stop if an obstacle is detected within 20 cm
  }

  // Read Bluetooth command (if available)
  if (Serial.available() > 0) {
    command = Serial.read();
    if (command == 'F') {
      moveForward();
    } else if (command == 'B') {
      moveBackward();
    } else if (command == 'L') {
      moveLeft();
    } else if (command == 'R') {
      moveRight();
    } else if (command == 'S') {
      stopMotors();
    }
  }
}

// Function to move the robot forward
void moveForward() {
  motor_left_front.setSpeed(255);
  motor_right_front.setSpeed(255);
  motor_left_back.setSpeed(255);
  motor_right_back.setSpeed(255);
  motor_left_front.run(FORWARD);
  motor_right_front.run(FORWARD);
  motor_left_back.run(FORWARD);
  motor_right_back.run(FORWARD);
}

// Function to move the robot backward
void moveBackward() {
  motor_left_front.setSpeed(255);
  motor_right_front.setSpeed(255);
  motor_left_back.setSpeed(255);
  motor_right_back.setSpeed(255);
  motor_left_front.run(BACKWARD);
  motor_right_front.run(BACKWARD);
  motor_left_back.run(BACKWARD);
  motor_right_back.run(BACKWARD);
}

// Function to move the robot left
void moveLeft() {
  motor_left_front.setSpeed(255);
  motor_right_front.setSpeed(255);
  motor_left_back.setSpeed(255);
  motor_right_back.setSpeed(255);
  motor_left_front.run(BACKWARD);
  motor_right_front.run(FORWARD);
  motor_left_back.run(BACKWARD);
  motor_right_back.run(FORWARD);
}

// Function to move the robot right
void moveRight() {
  motor_left_front.setSpeed(255);
  motor_right_front.setSpeed(255);
  motor_left_back.setSpeed(255);
  motor_right_back.setSpeed(255);
  motor_left_front.run(FORWARD);
  motor_right_front.run(BACKWARD);
  motor_left_back.run(FORWARD);
  motor_right_back.run(BACKWARD);
}

// Function to stop the robot
void stopMotors() {
  motor_left_front.setSpeed(0);
  motor_right_front.setSpeed(0);
  motor_left_back.setSpeed(0);
  motor_right_back.setSpeed(0);
  motor_left_front.run(RELEASE);
  motor_right_front.run(RELEASE);
  motor_left_back.run(RELEASE);
  motor_right_back.run(RELEASE);
}

// Function to measure distance using ultrasonic sensor
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = (duration / 2) * 0.0344;
  return distance;
}
