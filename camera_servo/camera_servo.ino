#include <Servo.h>

Servo servo;
int servoPin = A0;  // Servo pin connected to Arduino
float currentPos = 90;  // Initial position of the servo (centered)
float lastPos = 90;     // Last known position of the servo
float offset = 0;       // Current face offset
bool sweepingForward = true; // Flag to track sweeping direction

void setup() {
  servo.attach(servoPin);  // Attach the servo to pin A0
  servo.write(currentPos); // Set initial position
  Serial.begin(9600);      // Begin serial communication
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read incoming data from the serial port
    
    if (command == "scan") {
      // Keep scanning, servo keeps moving back and forth
      sweepServo();
    }
    else {
      // Move the servo based on the face position data
      offset = command.toFloat();
      moveServoSmoothly();
    }
  }
}

void sweepServo() {
  // Continuous sweeping from 0 to 180 degrees
  static unsigned long lastSweepTime = 0;  // Variable for time tracking
  unsigned long currentTime = millis();
  
  if (currentTime - lastSweepTime > 20) {  // Slow down the sweeping
    // Move the servo by 0.5 degrees in the current direction
    if (sweepingForward) {
      currentPos += 5;  // Move forward (0 to 180)
      if (currentPos >= 180) {
        sweepingForward = false;  // Change direction when reaching 180
      }
    } else {
      currentPos -= 5;  // Move backward (180 to 0)
      if (currentPos <= 0) {
        sweepingForward = true;  // Change direction when reaching 0
      }
    }

    servo.write(currentPos);  // Move the servo
    lastSweepTime = currentTime;
  }
}

void moveServoSmoothly() {
  // Calculate the new servo position based on the face offset
  int targetPos = 90 + (offset * 45);  // Scale the offset to a range of servo movement
  targetPos = constrain(targetPos, 0, 180);  // Ensure the position is within the servo range

  // Move smoothly towards the target position
  if (abs(targetPos - currentPos) > 1) {  // Smooth movement with gradual steps
    currentPos += (targetPos - currentPos) / 6;  // Smaller steps for smoother motion
    servo.write(currentPos);
    delay(15);  // Small delay to smooth out the movement
  } else {
    currentPos = targetPos;  // Snap to target position if close enough
    servo.write(currentPos);
  }
}
