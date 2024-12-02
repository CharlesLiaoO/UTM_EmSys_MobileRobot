#include <Arduino.h>
// #include <string.h>

// mcu input pins for joystick
const int joystick_Vert_Pin = 36;
const int joystick_Horz_Pin = 39;

// mcu output pins to motor driver input
const int motor1_In1 = 19;
const int motor1_In2 = 5;
const int motor1_PWM = 18;
const int motor2_In1 = 4;
const int motor2_In2 = 17;
const int motor2_PWM = 16;

// mcu input pins for motor's encoder
const int encoder1_C = 2;
const int encoder1_D = 15;
const int encoder2_C = 8;
const int encoder2_D = 7;

// Variables for velocity, position
int encoder1_Count = 0;
int encoder2_Count = 0;
int encoder1_Count_b = 0;
int encoder2_Count_b = 0;

unsigned long prevTime = 0;      // To calculate elapsed time
const int encoder_slots = 20;     // Number of slots in encoder disk
const float wheelDiameter = 0.020;      // Wheel diameter in meter
const float wheelBase = 0.15;           // Distance between wheels in meter

double linearVelocity = 0;
double angularVelocity = 0;
double linearVelocity_b = 0;
double angularVelocity_b = 0;

double posX = 0;  // robot's position X in meter
double posY = 0;
double heading = 90.0 * PI/180;  // robot's Heading angle in radian

// Interrupt service routines for encoder counting
const int encoder_dt = 10;  // use 1 in physical project
void IRAM_ATTR encoder1_ISR() {
  if (digitalRead(encoder1_D))
    encoder1_Count = encoder1_Count + encoder_dt;
  else
    encoder1_Count = encoder1_Count - encoder_dt;
}

void IRAM_ATTR encoder2_ISR() {
  if (digitalRead(encoder2_D))
    encoder2_Count = encoder2_Count + encoder_dt;
  else
    encoder2_Count = encoder2_Count - encoder_dt;
}

// Function to set motor direction and speed
void setMotorSpeed(int motor, float speed) {
  float pwmSpeed = abs(speed);
  float simEncode = 10 * pwmSpeed / 255;
  float simLedSpeed = pwmSpeed == 255 ? 255 : pwmSpeed * 0.1;

  if (motor == 1) { // Motor 1
    if (speed == 0) {
      digitalWrite(motor1_In1, LOW);
      digitalWrite(motor1_In2, LOW);
    } else if (speed > 0) {
      digitalWrite(motor1_In1, HIGH);
      digitalWrite(motor1_In2, LOW);
      encoder1_Count += simEncode;
    } else {
      digitalWrite(motor1_In1, LOW);
      digitalWrite(motor1_In2, HIGH);
      encoder1_Count -= simEncode;
    }
    analogWrite(motor1_PWM, simLedSpeed);

  } else if (motor == 2) { // Motor 2
    if (speed == 0) {
      digitalWrite(motor2_In1, LOW);
      digitalWrite(motor2_In2, LOW);
    } else if (speed > 0) {
      digitalWrite(motor2_In1, HIGH);
      digitalWrite(motor2_In2, LOW);
      encoder2_Count += simEncode;
    } else {
      digitalWrite(motor2_In1, LOW);
      digitalWrite(motor2_In2, HIGH);
      encoder2_Count -= simEncode;
    }
    analogWrite(motor2_PWM, simLedSpeed);
  }
}

// Function to calculate velocity and position
void calculateOdometry() {
  // Calculate time elapsed
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0; // seconds
  prevTime = currentTime;

  double dt1 = encoder1_Count - encoder1_Count_b;
  double dt2 = encoder2_Count - encoder2_Count_b;
  encoder1_Count_b = encoder1_Count;
  encoder2_Count_b = encoder2_Count;

  // Calculate wheel speeds (m/s)
  static double distPerCount = (PI * wheelDiameter) / encoder_slots * 0.1 * 10;  // 0.1 gear rate, multiply 10 for quick demonstrating
  double motorSpeed1 = (dt1 * distPerCount) / deltaTime;
  double motorSpeed2 = (dt2 * distPerCount) / deltaTime;

  // Calculate linear and angular velocity
  linearVelocity = (motorSpeed1 + motorSpeed2) / 2;
  angularVelocity = (motorSpeed2 - motorSpeed1) / wheelBase;

  if (linearVelocity_b == linearVelocity && angularVelocity_b == angularVelocity) {
    return;
  }
  linearVelocity_b = linearVelocity;
  angularVelocity_b = angularVelocity;

  if (0 == linearVelocity && 0 == angularVelocity) {
    return;
  }

  // Update robot's position
  // Serial.printf(): float can only format with %f, not also with %d!
  // Serial.printf("\n"): "\n" not cross-platform, needs \r\n in windows...
  // Serial.printf("test newline\r\n");
  double linearVelocity_x = linearVelocity * cos(heading);  // cos/sin() is in radian!
  double linearVelocity_y = linearVelocity * sin(heading);
  double heading_degP = heading * 180/PI;
  // Serial.printf("m1=%.3f, m2=%.3f, h=%.3f, vx=%.3f, vy=%.3f -- ", motorSpeed1, motorSpeed2, heading_degP, linearVelocity_x, linearVelocity_y);
  posX += linearVelocity_x * deltaTime;
  posY += linearVelocity_y * deltaTime;

  heading += angularVelocity * deltaTime;

  double angularVelocity_deg = angularVelocity * 180/PI;
  double heading_deg = heading * 180/PI;

  // Print speed, position
  Serial.printf("%.3fs -- Vel: lin=%.3f, ang=%.3f; Pos: x, y, h = %.3f, %.3f, %.3f\r\n", deltaTime, linearVelocity, angularVelocity_deg, posX, posY, heading_deg);
}

void ctrlSpeed() {
  // int vert = (analogRead(Joystick_Vert_Pin));
  // int horz = (analogRead(Joystick_Horz_Pin));
  int vert = map(analogRead(joystick_Vert_Pin), 0, 4095, -255, 255);  // map to foreward speed
  int horz = map(analogRead(joystick_Horz_Pin), 0, 4095, -125, 125);  // map to rotate speed, not turn speed

  if (horz == 0) {
    setMotorSpeed(1, vert);
    setMotorSpeed(2, vert);
  } else if (horz > 0) {    // left
    if (vert == 0) {
      setMotorSpeed(1, -horz);  // use rotate speed
      setMotorSpeed(2, horz);
    } else {
      setMotorSpeed(1, vert/2);   // use different vertical speed on two wheel to make sure for turn
      setMotorSpeed(2, vert);
    }
  } else {    // horz < 0    // right
    if (vert == 0) {
      setMotorSpeed(1, -horz);
      setMotorSpeed(2, horz);
    } else {
      setMotorSpeed(1, vert);
      setMotorSpeed(2, vert/2);
    }
  }
}

void setup() {
  pinMode(joystick_Vert_Pin, INPUT);
  pinMode(joystick_Horz_Pin, INPUT);

  // Motor pins setup
  pinMode(motor1_In1, OUTPUT);
  pinMode(motor1_In2, OUTPUT);
  pinMode(motor2_In1, OUTPUT);
  pinMode(motor2_In2, OUTPUT);

  // Encoder pins setup
  pinMode(encoder1_C, INPUT);    // INPUT_PULLUP
  pinMode(encoder1_D, INPUT);
  pinMode(encoder2_C, INPUT);
  pinMode(encoder2_D, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1_C), encoder1_ISR, FALLING);    //RISING
  attachInterrupt(digitalPinToInterrupt(encoder2_C), encoder2_ISR, FALLING);

  // ledcAttachPin(motor1_PWM, ch);  only ok in PlatformIO environment
  // ledcAttach()  only ok in websit Wokwi

  Serial.begin(115200); // For debugging output
  prevTime = millis();    // Initialize time

  Serial.println("---- setup finished ----");
};

void loop() {
  ctrlSpeed();

  calculateOdometry();

  delay(100); // Adjust for desired loop rate
}
