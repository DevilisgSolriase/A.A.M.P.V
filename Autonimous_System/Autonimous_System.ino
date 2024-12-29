#include <IRremote.h>
#include <Servo.h>

// Pin Definitions
#define Lpwm_pin 5  // Pin controlling speed - ENA
#define Rpwm_pin 6  // Pin controlling speed - ENB

// Pin Assignments
int pinLS = 3;
int pinMS = 12;
int pinRS = 11;
int pinIR = 9;
int pinLB = 2;
int pinLF = 4;
int pinRB = 7;
int pinRF = 8;

//mode value : 1- autonomous 2- manual
char mode = '2';

//light sensor values: if 0 clear if 1 there is a dip in the surface
unsigned char left_ls = 0;
unsigned char middle_ls = 0;
unsigned char right_ls = 0;
bool pit_detected = false;

// Global Variables
Servo myservo;
IRrecv irrecv(pinIR);
decode_results results;
volatile int DL, DM, DR;
unsigned char pwm_val = 150;

// Timing Variables for Non-blocking Operations
unsigned long previousServoMillis = 0;
const long servoInterval = 20; // Interval for servo updates

unsigned long previousObstacleMillis = 0;
const long obstacleInterval = 200; // Interval for obstacle checks

// Functions
float checkdistance() {
  digitalWrite(A1, LOW);
  delayMicroseconds(2);
  digitalWrite(A1, HIGH);
  delayMicroseconds(10);
  digitalWrite(A1, LOW);
  float distance = pulseIn(A0, HIGH, 30000) / 58.0; // Timeout after 30ms
  return distance;
}

void react(int pos, int DM) {
  stopp();  // Stop the robot
  Set_Speed(0);  // Stop motors
  delay(1000);  // Wait for the robot to stop

  // Check which half of the servo's sweep the obstacle is in
  if (pos <= 90) {  // Object detected in the left half of the servo sweep
    Serial.println("Obstacle on the left, turning right.");
    turnR();  // Turn right to avoid obstacle
    Set_Speed(200);  // Speed up to turn
    delay(200);  // Allow the robot to turn
    advance();  // Move forward after turning
  } else if (pos > 90) {  // Object detected in the right half of the servo sweep
    Serial.println("Obstacle on the right, turning left.");
    turnL();  // Turn left to avoid obstacle
    Set_Speed(200);  // Speed up to turn
    delay(200);  // Allow the robot to turn
    advance();  // Move forward after turning
  }
}



void auto_move() {
  static int pos = 45;  // Starting servo position (middle)
  const int step = 4;   // Small step for slow rotation
  const int threshold = 30; // Distance threshold for obstacle detection

  // Slowly rotate the servo to scan for obstacles
  for (pos = 45; pos <= 135; pos += step) {
    myservo.write(pos);  // Move the servo to the current position
    delay(100);  // Delay to allow servo to reach position
    int distance = checkdistance();  // Measure the distance

    Serial.print("Servo Position: ");
    Serial.print(pos);
    Serial.print(" | Distance: ");
    Serial.println(distance);

    // If an obstacle is detected, stop and decide the reaction
    if (distance < threshold) {
      Serial.println("Obstacle detected!");
      react(pos, distance);  // Run the function to decide direction
      return;  // Exit the function if obstacle is detected
    }
  }

  // Reverse the direction of servo scan after reaching 135 degrees
  for (pos = 135; pos >= 45; pos -= step) {
    myservo.write(pos);  // Move the servo to the current position
    delay(100);  // Delay to allow servo to reach position
    int distance = checkdistance();  // Measure the distance

    Serial.print("Servo Position: ");
    Serial.print(pos);
    Serial.print(" | Distance: ");
    Serial.println(distance);

    // If an obstacle is detected, stop and decide the reaction
    if (distance < threshold) {
      Serial.println("Obstacle detected!");
      react(pos, distance);  // Run the function to decide direction
      return;  // Exit the function if obstacle is detected
    }
  }
}

void Detect_obstacle_distance() {
  myservo.write(160);
  for (int i = 0; i < 3; i++) DL = checkdistance();
  delay(100);
  myservo.write(20);
  for (int i = 0; i < 3; i++) DR = checkdistance();
  delay(100);
}

void setup() {
  myservo.attach(A2);
  pinMode(A1, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(pinLB, OUTPUT);
  pinMode(pinLF, OUTPUT);
  pinMode(pinRB, OUTPUT);
  pinMode(pinRF, OUTPUT);
  pinMode(Lpwm_pin, OUTPUT);
  pinMode(Rpwm_pin, OUTPUT);
  pinMode(pinLS, INPUT);
  pinMode(pinMS, INPUT);
  pinMode(pinRS, INPUT);
  DL = 0;
  DM = 0;
  DR = 0;
  Serial.begin(9600);
  irrecv.enableIRIn();
  myservo.write(90);
}

void key_Control(char input) {
  switch (input) {
    case 'w':
      Set_Speed(pwm_val);
      advance();  // Move forward
      break;
    case 's':
      Set_Speed(pwm_val);
      back();  // Move backward
      break;
    case 'a':
      Set_Speed(pwm_val);
      turnL();  // Turn left
      break;
    case 'd':
      Set_Speed(pwm_val);
      turnR();  // Turn right
      break;
    case 'x':
      stopp();  // Stop
      break;
    default:
      break;
  }
}

// void pit_detection() {
//    if (middle_ls == 1 && left_ls == 1) {
//     if (!pit_detected) {
//       Serial.println("Pit on the left side detected");
//       back();
//       delay(500); // Allow robot to back away
//       turnR();
//       delay(500);
//       stopp();
//       pit_detected = true;  // Prevent re-triggering
//     }
//   } else if (middle_ls == 1 && right_ls == 1) {
//     if (!pit_detected) {
//       Serial.println("Pit on the right side detected");
//       back();
//       delay(500);
//       turnL();
//       delay(500);
//       stopp();
//       pit_detected = true;
//     } else {
//       pit_detected = false;
//     }
//   }
// }

void loop() {
  unsigned long currentMillis = millis();

  // Handle Serial input for mode switching
  if (Serial.available()) {
    char input = Serial.read();  // Read the character input from Serial Monitor
    input = toLowerCase(input);  // Convert to lowercase if needed, for case-insensitivity

    // Switch between modes when 'm' is pressed
    if (input == 'm') {
      if (mode == '1') {
        mode = '2';  // Switch to Autonomous mode
        Serial.println("Switched to Autonomous Mode");
      } else {
        mode = '1';  // Switch to IR control mode
        Serial.println("Switched to Manual Mode");
      }
    }

    if (mode == '1') {
      key_Control(input);  // Manual Mode controls
    }
  }

  if (mode == '2') {
    // Autonomous Mode (non-blocking)
    auto_move();
  }
}

// Motor Control Functions
void Set_Speed(unsigned char pwm) {
  analogWrite(Lpwm_pin, pwm);
  analogWrite(Rpwm_pin, pwm);
}

void advance() {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinLF, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW);
}

void turnR() {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
}

void turnL() {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
}

void stopp() {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, HIGH);
}

void back() {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
}
