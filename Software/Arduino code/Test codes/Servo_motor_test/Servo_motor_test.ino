#include <Servo.h>

// Create a Servo object for each servo
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Define the servo PWM pin numbers
int servoPin1 = 2;
int servoPin2 = 3;
int servoPin3 = 4;
int servoPin4 = 5;

void setup() {
  // Attach each servo to its corresponding pin
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);

  // Initially set each servo to 0 degrees steering angle
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  delay(5000); // Wait for 5 seconds

  // Move each servo to 90 degrees
  servo1.write(45);
  servo2.write(45);
  servo3.write(45);
  servo4.write(45);
  delay(5000); // Wait for 5 seconds

  // Move each servo to 180 degrees
  servo1.write(135);
  servo2.write(135);
  servo3.write(135);
  servo4.write(135);
  delay(5000); // Wait for 5 seconds

  // Return each servo to 0 degrees
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);

  delay(5000); // Wait for 5 seconds
}

void loop() {
  // Your loop code here (if needed)
}
