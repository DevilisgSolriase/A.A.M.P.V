#include <IRremote.h>
#include <Servo.h>

// Pin Definitions
#define Lpwm_pin 5  // Pin controlling speed - ENA
#define Rpwm_pin 6  // Pin controlling speed - ENB
#define IR_Go 0x00FF18E7
#define IR_Back 0x00FF4AB5
#define IR_Left 0x00FF10EF
#define IR_Right 0x00FF5AA5
#define IR_Stop 0x00FF38C7

// Operation Modes
unsigned int mode = 0x00FFA25D;  // Initial mode (controller or autonomous)

// Pin Assignments
int pinIR = 9;
int pinLB = 2;
int pinLF = 4;
int pinRB = 7;
int pinRF = 8;
int pos = 0;

// Global Variables
Servo myservo;
IRrecv irrecv(pinIR);
decode_results results;
volatile int DL, DM, DR;
unsigned char pwm_val = 150;
unsigned long Key;

// Functions
float checkdistance() {
  digitalWrite(A1, LOW);
  delayMicroseconds(2);
  digitalWrite(A1, HIGH);
  delayMicroseconds(10);
  digitalWrite(A1, LOW);
  float distance = pulseIn(A0, HIGH) / 58.00;
  delay(10);
  return distance;
}

void react(int pos, volatile int DM) {
  stopp();
  Set_Speed(0);
  delay(1000);
  if (pos >= 50 && pos <= 60) {
    turnL();
    Set_Speed(200);
    delay(200);
    advance();
    Set_Speed(200);
    return;
  } else if (pos >= 125 && pos <= 135) {
    turnR();
    Set_Speed(200);
    delay(200);
    advance();
    Set_Speed(200);
    return;
  }
  Detect_obstacle_distance();
  if (DL < 50 || DR < 50) {
    if (DL > DR) {
      myservo.write(90);
      turnL();
      Set_Speed(200);
      delay(200);
      advance();
      Set_Speed(200);
    } else {
      myservo.write(90);
      turnR();
      Set_Speed(200);
      delay(200);
      advance();
      Set_Speed(200);
    }
  } else {
    if (random(1, 10) > 5) {
      myservo.write(90);
      turnL();
      Set_Speed(200);
      delay(200);
      advance();
      Set_Speed(200);
    } else {
      myservo.write(90);
      turnR();
      Set_Speed(200);
      delay(200);
      advance();
      Set_Speed(200);
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

void handleIR() {
  if (irrecv.decode(&results)) {
    Key = results.value;
    Serial.println("IR scanned");
    Serial.println(Key, HEX);
    delay(50);  // Debounce delay
    if (Key == 0x00FFA25D || Key == 0x00FF629D) {
      mode = Key;
    }
    irrecv.resume();  // Ready to receive next signal
  }
}

void IR_Control() {
  handleIR();  // Ensure IR signals are read
  switch (Key) {
    case IR_Go:
      Set_Speed(pwm_val);
      advance();  // Move forward
      break;
    case IR_Back:
      Set_Speed(pwm_val);
      back();  // Move backward
      break;
    case IR_Left:
      Set_Speed(pwm_val);
      turnL();  // Turn left
      break;
    case IR_Right:
      Set_Speed(pwm_val);
      turnR();  // Turn right
      break;
    case IR_Stop:
      stopp();  // Stop
      break;
    default:
      break;
  }
}

void auto_move() {
  static int auto_pos = 45;
  static bool direction = true;  // Servo sweep direction
  static unsigned long auto_time = 0;

  handleIR();  // Ensure IR signals are read during autonomous mode

  if (millis() - auto_time > 20) {  // Non-blocking servo control
    if (direction) {
      auto_pos += 4;
      if (auto_pos >= 135) direction = false;
    } else {
      auto_pos -= 4;
      if (auto_pos <= 45) direction = true;
    }
    myservo.write(auto_pos);
    DM = checkdistance();
    if (DM < 30) {
      react(auto_pos, DM);
    } else {
      advance();
      Set_Speed(130);
    }
    auto_time = millis();
  }
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
  DL = 0;
  DM = 0;
  DR = 0;
  Serial.begin(9600);
  irrecv.enableIRIn();
  myservo.write(90);
}

void loop() {
  handleIR();  // Always check for IR input
  switch (mode) {
    case 0x00FFA25D:  // IR control mode
      IR_Control();
      break;
    case 0x00FF629D:  // Autonomous mode
      auto_move();
      break;
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
