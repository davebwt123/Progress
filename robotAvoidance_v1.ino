#include <ESP32Servo.h>
#include <PS4Controller.h>

//Servo + Sensor
#define TRIG_PIN 26
#define ECHO_PIN 27
#define SERVO_PIN 25

// Motor
#define IN1 14  // Left Motor Forward
#define IN2 12  // Left Motor Backward
#define IN3 32  // Right Motor Forward
#define IN4 33  // Right Motor Backward

//controller PS4
float Vx = 0.0, Vy = 0.0, w = 0.0;
float v1 = 0.0, v2 = 0.0, v3 = 0.0, lifter = 0.0;
float shooterSpeed = 0.0;
float conveyorSpeed = 0.0;
int mode = 0;
bool lastTriangleState = false;
bool conveyorUpActive = false;
bool conveyorDownActive = false;
unsigned long conveyorTimer = 0;
bool lastL1State = false;
bool lastL2State = false;

Servo myServo;

//Servo
int distanceThreshold = 10; // cm
int frontAngle = 90;
int rightAngle = 0;
int leftAngle  = 180;
long rightDistance = 0;
long leftDistance = 0;


//Motor
// #define MAX_SPEED 200         // Maximum motor speed (0-255)
// #define TURN_SPEED 180        // Speed during turns
#define BACKUP_TIME 500       // Time to backup in milliseconds
#define TURN_TIME 400         // Time to turn in milliseconds

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

void modeAutonomous();
void modeUserControl();
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void stopMotors();


void setup() {
  Serial.begin(115200);
  myServo.attach(SERVO_PIN);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  myServo.write(frontAngle);
  delay(500);

  //motor
  // Initialize motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // PS4.begin("10:68:38:3f:26:9e"); // dari robot 2 
  // Serial.println("PS4 ready to connect!");

  Serial.println("Robot initialized!");
  Serial.println("Starting obstacle avoidance...");  
}

void loop() {
  long frontDistance = getDistance();
  Serial.print("Jarak:  ");
  Serial.println(frontDistance);



  if (frontDistance < distanceThreshold) {
   
    //Berhenti jika sudah di jarak bahaya
    stopMotors();
    delay(500);
    
    Serial.println("Ada obstacles/rintangan");
    delay(500);

    myServo.write(rightAngle);
    delay(1000);
    long rightDistance = getDistance();
    Serial.print("Jarak Kanan: ");
    Serial.println(rightDistance);

    myServo.write(leftAngle);
    delay(1000);
    long leftDistance = getDistance();
    Serial.print("Jarak Kiri: ");
    Serial.println(leftDistance);

    myServo.write(frontAngle);
    delay(500);

    moveBackward();
    delay(BACKUP_TIME);
    stopMotors();
    delay(200);

    // belok kanan kalo kanan lebih selo
    if (rightDistance > distanceThreshold && rightDistance > leftDistance) {
      Serial.println("Belok Kanan");
      turnRight();
      delay(TURN_TIME);
    }

    // Belok kiri kalo kiri lebih selo
    else if (leftDistance > distanceThreshold && leftDistance > rightDistance) {
      Serial.println("Belok Kiri");
      turnLeft();
      delay(TURN_TIME);
    } 
    
    else {
      Serial.println("Both sides blocked!");
    }
  }else{
    Serial.println("Jalur Aman - Lanjut Maju");
    moveForward();
  }

delay(50);
}

// Move forward
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Move backward
void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Turn right
void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Turn left
void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Stop all motors
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

}


