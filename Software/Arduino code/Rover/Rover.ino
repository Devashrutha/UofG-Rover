// Include required libraries for peripherals
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#define BT Serial3
// Define the motor encoder pins
#define ENCODER1_INTERRUPT_PIN 40
#define ENCODER2_INTERRUPT_PIN 42
// The number of slits in the encoder disk
#define ENCODER_N 20
// Timing variables for the motor encoder interrupt functions
unsigned long T1 = 0, T2 = 0, Ta;
unsigned long T3 = 0, T4 = 0, Tb;
bool MeasDone1 = 0;
bool MeasDone2 = 0;
// Motor encoder RPM variables
int Motor1_RPM = 0;
int Motor2_RPM = 0;
// Create mpu object for the MPU6050 IMU
MPU6050 mpu;
// yaw/pitch/roll angles (in degrees) calculated from the quaternions coming from the FIFO
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13
#define IMU_INTERRUPT_PIN 38
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer
// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
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
// Create a Servo object for each servo
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
// Define the servo PWM pin numbers
int servoPin1 = 8;
int servoPin2 = 9;
int servoPin3 = 10;
int servoPin4 = 11;
//Initial servo motor angle in degrees
int se1Angle = 80;
int se2Angle = 88;
int se3Angle = 97;
int se4Angle = 87;
// s is the required speed in percentage 0 to 100%
// s=68 is the maximum while turning
int s = 0;
float speed1, speed2 = 0;
float speed1PWM, speed2PWM = 0;
// alpha is the heading angle, alpha1 and alpha2 represent the inner and outer wheel servos
float alpha, alpha1, alpha2 = 0;
float alpha_rad;
// On point turning angle calculated from equation 9
float alpha_on_point = 43.73;
// Inputs received from the keyboard
char inp;
float wheelbase = 460.0;
float trackwidth = 440.0;
float turning_radius;

unsigned long lastCommandTime = 0;         // Stores the last time a command was received
const unsigned long commandTimeout = 500;  // Timeout in milliseconds

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // put your setup code here, to run once:
  Serial.begin(115200);
  BT.begin(115200);
  

  //initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(IMU_INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // gyro offsets
  mpu.setXGyroOffset(500);
  mpu.setYGyroOffset(0.27);
  mpu.setZGyroOffset(3.18);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
    // Calibration time
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // Enable Arduino DUE interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(IMU_INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  // Initialize the motor encoder
  pinMode(ENCODER1_INTERRUPT_PIN, INPUT);
  pinMode(ENCODER2_INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_INTERRUPT_PIN), ISR_1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_INTERRUPT_PIN), ISR_2, RISING);

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
  // Attach each servo to its corresponding pin
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  //Set the initial angles to move straight
  servo1.write(se1Angle);
  servo2.write(se2Angle);
  servo3.write(se3Angle);
  servo4.write(se4Angle);
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void loop() {

  Motor1_RPM = (60000000) / (Ta * ENCODER_N);
  Motor2_RPM = (60000000) / (Tb * ENCODER_N);
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print(" YAW:\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print(" ROL:\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.print(" PTH:\t");
    Serial.println(ypr[1] * 180 / M_PI);
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
  // Print the RPM data and move the cursor to the begining of the serial monitor
  Serial.print(" RPM1:\t");
  Serial.print(Motor1_RPM);
  Serial.print(" RPM2:\t");
  Serial.println(Motor2_RPM);
  // Control logic for moving the rover
  if (BT.available()) {
    inp = BT.read();
    lastCommandTime = millis();
    switch (inp) {
      case 'w':  // Move forward
      case 'W':
        motorSpeed();
        moveForward(speed1PWM, speed2PWM);
        break;
      case 's':  // Move back
      case 'S':
        motorSpeed();
        moveBackward(speed1PWM, speed2PWM);
        break;
      case 'd':  // Turn right
      case 'D':
        alpha += 1;                  // Increase alpha for right turn
        if (alpha > 22) alpha = 22;  // Limit max turn angle
        moveLeft();
        break;
      case 'a':  // Turn left
      case 'A':
        alpha -= 1;                    // Decrease alpha for left turn
        if (alpha < -22) alpha = -22;  // Limit max turn angle
        moveRight();
        break;
      case 'r':  // Rotate on point
      case 'R':
        motorSpeed();
        rotateOnPoint(speed1PWM, speed2PWM);
        break;
      case 'x':  // Stop all movement
      case 'X':
        stopMovement();
        break;
    }
  }  // Check for command timeout to stop the rover if no command is received
  if (millis() - lastCommandTime > commandTimeout) {
    stopMovement();
  }
}

void moveForward(float speedPWM1, float speedPWM2)  // Forward
{
  analogWrite(en1, speedPWM1);
  digitalWrite(in1a, HIGH);
  digitalWrite(in1b, LOW);
  analogWrite(en2, speedPWM2);
  digitalWrite(in2a, HIGH);
  digitalWrite(in2b, LOW);
  analogWrite(en3, speedPWM1);
  digitalWrite(in3a, LOW);
  digitalWrite(in3b, HIGH);
  analogWrite(en4, speedPWM2);
  digitalWrite(in4a, LOW);
  digitalWrite(in4b, HIGH);
  analogWrite(en5, speedPWM1);
  digitalWrite(in5a, HIGH);
  digitalWrite(in5b, LOW);
  analogWrite(en6, speedPWM2);
  digitalWrite(in6a, HIGH);
  digitalWrite(in6b, LOW);
}
void moveBackward(float speedPWM1, float speedPWM2)  // Backward
{
  analogWrite(en1, speedPWM1);
  digitalWrite(in1a, LOW);
  digitalWrite(in1b, HIGH);
  analogWrite(en2, speedPWM2);
  digitalWrite(in2a, LOW);
  digitalWrite(in2b, HIGH);
  analogWrite(en3, speedPWM1);
  digitalWrite(in3a, HIGH);
  digitalWrite(in3b, LOW);
  analogWrite(en4, speedPWM2);
  digitalWrite(in4a, HIGH);
  digitalWrite(in4b, LOW);
  analogWrite(en5, speedPWM1);
  digitalWrite(in5a, LOW);
  digitalWrite(in5b, HIGH);
  analogWrite(en6, speedPWM2);
  digitalWrite(in6a, LOW);
  digitalWrite(in6b, HIGH);
}

void moveRight() {
  servoAngle();  // Update alpha1 and alpha2 based on current alpha

  // Adjust servo angles for turning right
  servo1.write(se1Angle + alpha1);
  servo2.write(se2Angle + alpha2);
  servo3.write(se3Angle - alpha1);
  servo4.write(se4Angle - alpha2);
}

void moveLeft() {
  servoAngle();  // Update alpha1 and alpha2 based on current alpha

  // Adjust servo angles for turning left
  servo1.write(se1Angle + alpha1);
  servo2.write(se2Angle + alpha2);
  servo3.write(se3Angle - alpha1);
  servo4.write(se4Angle - alpha2);
}
void rotateOnPoint(float speedPWM1, float speedPWM2) {
  // Adjust servo angles for turning on point
  servo1.write(se1Angle + alpha_on_point);
  servo2.write(se2Angle - alpha_on_point);
  servo3.write(se3Angle - alpha_on_point);
  servo4.write(se4Angle + alpha_on_point);

  analogWrite(en1, speedPWM1);
  digitalWrite(in1a, HIGH);
  digitalWrite(in1b, LOW);
  analogWrite(en2, speedPWM2);
  digitalWrite(in2a, LOW);
  digitalWrite(in2b, HIGH);
  analogWrite(en3, speedPWM1);
  digitalWrite(in3a, LOW);
  digitalWrite(in3b, HIGH);
  analogWrite(en4, speedPWM2);
  digitalWrite(in4a, HIGH);
  digitalWrite(in4b, LOW);
  analogWrite(en5, speedPWM1);
  digitalWrite(in5a, HIGH);
  digitalWrite(in5b, LOW);
  analogWrite(en6, speedPWM2);
  digitalWrite(in6a, LOW);
  digitalWrite(in6b, HIGH);
}
void stopMovement() {
  analogWrite(en1, 0);
  digitalWrite(in1a, LOW);
  digitalWrite(in1b, LOW);
  analogWrite(en2, 0);
  digitalWrite(in2a, LOW);
  digitalWrite(in2b, LOW);
  analogWrite(en3, 0);
  digitalWrite(in3a, LOW);
  digitalWrite(in3b, LOW);
  analogWrite(en4, 0);
  digitalWrite(in4a, LOW);
  digitalWrite(in4b, LOW);
  analogWrite(en5, 0);
  digitalWrite(in5a, LOW);
  digitalWrite(in5b, LOW);
  analogWrite(en6, 0);
  digitalWrite(in6a, LOW);
  digitalWrite(in6b, LOW);
}

void motorSpeed() {

  if (alpha > 0.0) {  // Turning right
    s = 50;           // speed set to 30%
    turning_radius = (wheelbase / (2 * tan(abs(alpha) * PI / 180)));

    speed1 = s * sqrt(pow((0.5 * wheelbase), 2) + pow((0.5 * trackwidth - turning_radius), 2)) / turning_radius;
    speed2 = s * sqrt(pow((0.5 * wheelbase), 2) + pow((0.5 * trackwidth + turning_radius), 2)) / turning_radius;


    speed1PWM = map(round(speed1), 0, 100, 0, 255);
    speed2PWM = map(round(speed2), 0, 100, 0, 255);
  }

  else if (alpha < 0.0) {  // Turning left
    s = 50;                // speed set to 30%
    turning_radius = (wheelbase / (2 * tan(abs(alpha) * PI / 180)));

    speed1 = s * sqrt(pow((0.5 * wheelbase), 2) + pow((0.5 * trackwidth - turning_radius), 2)) / turning_radius;
    speed2 = s * sqrt(pow((0.5 * wheelbase), 2) + pow((0.5 * trackwidth + turning_radius), 2)) / turning_radius;


    speed1PWM = map(round(speed2), 0, 100, 0, 255);
    speed2PWM = map(round(speed1), 0, 100, 0, 255);
  }

  else {     // Moving forward and back
    s = 50;  // speed set to 30%
    speed1 = speed2 = s;
    speed1PWM = map(round(s), 0, 100, 0, 255);
    speed2PWM = map(round(s), 0, 100, 0, 255);
  }
}

void servoAngle() {

  alpha_rad = alpha * PI / 180;  // Convert degrees to radians
  //Calculate the servo angles in degrees
  alpha1 = round(atan(wheelbase / ((wheelbase / tan(alpha_rad)) + trackwidth)) * 180 / PI);
  alpha2 = round(atan(wheelbase / ((wheelbase / tan(alpha_rad)) - trackwidth)) * 180 / PI);
}
// Motor encoder interrupt functions
void ISR_1(void) {
  if (MeasDone1) {
    T2 = micros();
    Ta = T2 - T1;
    MeasDone1 = 0;
  } else {
    T1 = micros();
    MeasDone1 = 1;
  }
}

void ISR_2(void) {
  if (MeasDone2) {
    T4 = micros();
    Tb = T4 - T3;
    MeasDone2 = 0;
  } else {
    T3 = micros();
    MeasDone2 = 1;
  }
}