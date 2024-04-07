// Define the pins used on the Arduino Due.
// Enable pins for controlling the speed of the motor(enX).
#define en1 2
#define en2 3
#define en3 4
#define en4 5
#define en5 6
#define en6 7
// Input pins of the motor(inX and inY).
// Motor 1
#define in1a 30
#define in1b 32
// Motor 2
#define in2a 34
#define in2b 36
// Motor 3
#define in3a 22
#define in3b 24
// Motor 4
#define in4a 26
#define in4b 28
// Motor 5
#define in5a 23
#define in5b 25
// Motor 6
#define in6a 27
#define in6b 29

void setup() {
  // Initialize the pins as either INPUT or OUTPUT.
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(en3, OUTPUT);
  pinMode(en4, OUTPUT);
  pinMode(en5, OUTPUT);
  pinMode(en6, OUTPUT);

  pinMode(in1a, OUTPUT);
  pinMode(in1b, OUTPUT);
  pinMode(in2a, OUTPUT);
  pinMode(in2b, OUTPUT);
  pinMode(in3a, OUTPUT);
  pinMode(in3b, OUTPUT);
  pinMode(in4a, OUTPUT);
  pinMode(in4b, OUTPUT);
  pinMode(in5a, OUTPUT);
  pinMode(in5b, OUTPUT);
  pinMode(in6a, OUTPUT);
  pinMode(in6b, OUTPUT);
  // Setting the speed of the motors using PWM signal 0-off to 255-max.
  analogWrite(en1, 255);
  analogWrite(en2, 255);
  analogWrite(en3, 255);
  analogWrite(en4, 255);
  analogWrite(en5, 255);
  analogWrite(en6, 255);
  // Setting the input pins at the motor driver to HIGH or LOW based on the direction required.
  // HIGH&HIGH - off, LOW&LOW - off, HIGH&LOW - clockwise, LOW&HIGH - anticlockwise. 
  digitalWrite(in1a, HIGH);
  digitalWrite(in1b, LOW);

  digitalWrite(in2a, HIGH);
  digitalWrite(in2b, LOW);

  digitalWrite(in3a, HIGH);
  digitalWrite(in3b, LOW);

  digitalWrite(in4a, HIGH);
  digitalWrite(in4b, LOW);

  digitalWrite(in5a, HIGH);
  digitalWrite(in5b, LOW);

  digitalWrite(in6a, HIGH);
  digitalWrite(in6b, LOW);

  delay(5000);

  digitalWrite(in1a, LOW);
  digitalWrite(in1b, LOW);

  digitalWrite(in2a, LOW);
  digitalWrite(in2b, LOW);

  digitalWrite(in3a, LOW);
  digitalWrite(in3b, LOW);

  digitalWrite(in4a, LOW);
  digitalWrite(in4b, LOW);

  digitalWrite(in5a, LOW);
  digitalWrite(in5b, LOW);

  digitalWrite(in6a, LOW);
  digitalWrite(in6b, LOW);
}

void loop() {

}
