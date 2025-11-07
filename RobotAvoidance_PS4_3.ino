#include <ESP32Servo.h>
#include <PS4Controller.h>

//Servo + Sensor
#define TRIG_PIN 26
#define ECHO_PIN 27
#define SERVO_PIN 25

// Motor
#define IN1 14  // Left Motor Forward
#define IN2 13  // Left Motor Backward
#define IN3 32  // Right Motor Forward
#define IN4 33  // Right Motor Backward

// PWM Channels
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// Controller PS4
float Vx = 0.0, Vy = 0.0, w = 0.0;
float v1 = 0.0, v2 = 0.0, v3 = 0.0, lifter = 0.0;
float shooterSpeed = 0.0;
int mode = 0;
bool lastTriangleState = false;
bool lastL1State = false;
bool lastL2State = false;

// Global distance variable for use across functions
long frontDistance = 0;

Servo myServo;

// Servo
int distanceThreshold = 20; // cm
int frontAngle = 90;
int rightAngle = 180;
int leftAngle = 0;
long rightDistance = 0;
long leftDistance = 0;

// Motor
#define BACKUP_TIME 500       // Time to backup in milliseconds
#define TURN_TIME 400         // Time to turn in milliseconds
#define MAX_MOTOR_SPEED 200   // Maximum safe motor speed

// Function declarations
void setupPWM();
void modeAutonomous();
void modeUserControl();
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void stopMotors();
void moveWithSpeed(int leftSpeed, int rightSpeed);
long getDistance();

void setupPWM() {
  // Configure PWM for motor control using ledcAttach (newer ESP32 Arduino library)
  ledcAttach(IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(IN3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(IN4, PWM_FREQ, PWM_RESOLUTION);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  myServo.attach(SERVO_PIN);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  myServo.write(frontAngle);
  delay(500);

  // Initialize motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Setup PWM for motor control
  setupPWM();

  PS4.begin("10:68:38:3f:26:9e");
  Serial.println("PS4 ready to connect!");

  Serial.println("Robot initialized!");
  Serial.println("Starting obstacle avoidance...");  
}

void loop() {
  if (PS4.isConnected()) {
    modeUserControl();
  }
  else {
    modeAutonomous();
    Serial.println("Controller disconnected");
  }

  delay(50);
}

void modeUserControl() {
  if (PS4.isConnected()) {
    // Toggle mode with Triangle button
    bool currentTriangleState = PS4.Triangle();
    if (currentTriangleState && !lastTriangleState) {
      mode = (mode == 0) ? 1 : 0;
      Serial.print("Mode switched to: ");
      Serial.println(mode == 0 ? "Safe (50% speed)" : "Sport (100% speed)");
    }
    lastTriangleState = currentTriangleState;

    // Get raw joystick values
    int stickY = PS4.LStickY();  // Forward/Backward (-128 to 127)
    int stickX = PS4.RStickX();  // Rotation (-128 to 127)

    // Map to speed ranges
    int forwardSpeed = map(stickY, -128, 127, 255, -255);  // Inverted for intuitive control
    int turnSpeed = map(stickX, -128, 127, -255, 255);     // Map to motor speed range

    // Apply mode sensitivity
    if (mode == 0) {  // Safe mode - 50% speed
      forwardSpeed = forwardSpeed / 2;
      turnSpeed = turnSpeed / 2;
    }
    // Mode 1 is full speed (no division)

  
    // Calculate left and right motor speeds using differential drive
    int leftMotor = forwardSpeed + turnSpeed;
    int rightMotor = forwardSpeed - turnSpeed;

    // Constrain to valid range (-255 to 255)
    leftMotor = constrain(leftMotor, -255, 255);
    rightMotor = constrain(rightMotor, -255, 255);

    // deadzone (ignore small stick movements)
    if (abs(stickY) < 15 && abs(stickX) < 15) {
      leftMotor = 0;
      rightMotor = 0;
    }

    moveWithSpeed(leftMotor, rightMotor);

    // Debug output
    Serial.print("LStick: ");
    Serial.print(stickY);
    Serial.print(" | RStick: ");
    Serial.print(stickX);
    Serial.print(" | Left: ");
    Serial.print(leftMotor);
    Serial.print(" | Right: ");
    Serial.println(rightMotor);
  }
}

void modeAutonomous() {
  frontDistance = getDistance();
  Serial.print("Jarak: ");
  Serial.println(frontDistance);
  delay(10);

  if (frontDistance < distanceThreshold) {
    // Stop if obstacle detected at danger distance
    stopMotors();
    delay(500);
    
    Serial.println("Ada obstacles/rintangan");
    delay(100);

    // Check right side
    myServo.write(rightAngle);
    delay(500);
    rightDistance = getDistance();
    Serial.print("Jarak Kanan: ");
    Serial.println(rightDistance);

    // Check left side
    myServo.write(leftAngle);
    delay(500);
    leftDistance = getDistance();
    Serial.print("Jarak Kiri: ");
    Serial.println(leftDistance);

    // Return servo to front
    myServo.write(frontAngle);
    delay(100);

    // Backup
    moveBackward();
    delay(BACKUP_TIME);
    stopMotors();
    delay(500);

    // Decide which way to turn
    if (rightDistance > distanceThreshold && rightDistance > leftDistance) {
      Serial.println("Belok Kanan");
      turnRight();
      delay(TURN_TIME);
    }
    else if (leftDistance > distanceThreshold && leftDistance > rightDistance) {
      Serial.println("Belok Kiri");
      turnLeft();
      delay(TURN_TIME);
    } 
    else {
      Serial.println("Both sides blocked!");
    }
    
    stopMotors();
  }
  else {
    Serial.println("Jalur Aman - Lanjut Maju");
    moveForward();
  }
}

// Get distance from ultrasonic sensor
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

// Move with PWM speed control
void moveWithSpeed(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed > 0) {
    ledcWrite(IN1, leftSpeed);   // IN1 - forward
    ledcWrite(IN2, 0);           // IN2 - backward
  } else {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, abs(leftSpeed));  // IN2 - backward
  }
  
  // Right motor
  if (rightSpeed > 0) {
    ledcWrite(IN3, rightSpeed);  // IN3 - forward
    ledcWrite(IN4, 0);           // IN4 - backward
  } else {
    ledcWrite(IN3, 0);
    ledcWrite(IN4, abs(rightSpeed)); // IN4 - backward
  }
}

// Move forward with fixed speed
void moveForward() {
  moveWithSpeed(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
}

// Move backward with fixed speed
void moveBackward() {
  moveWithSpeed(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
}

// Turn right with fixed speed
void turnRight() {
  moveWithSpeed(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
}

// Turn left with fixed speed
void turnLeft() {
  moveWithSpeed(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
}

// Stop all motors
void stopMotors() {
  ledcWrite(IN1, 0);
  ledcWrite(IN2, 0);
  ledcWrite(IN3, 0);
  ledcWrite(IN4, 0);
}