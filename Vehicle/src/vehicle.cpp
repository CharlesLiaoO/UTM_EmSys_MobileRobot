#include <Arduino.h>
// #include <string.h>
#include <ESP8266WiFi.h>
#include "PID.h"
// #include "PID_v1.h"

// WIFI AP & tcp server
const char* ssid = "lch-EmSys_Vehicle";
const char* password = "lch12321";
const int port = 2020;
WiFiServer server(port);

// mcu output pins to motor driver input
// 8266 NodeMCU V3: GPIO 14: HSCLK: pulse at moment after boot..
//    gpio02: connect to blue led
// the following pin define seems to be the only-working one...
const int motor1_In1 = 16;  // D0 ok
// const int motor1_In1 = 10;  // S3  cause reboot
// const int motor1_In2 = 0;  // D3  cannot output 3.3V, only 2.4... why??
const int motor1_In2 = 2;  // D4  on board blue led, ok
// const int motor1_In2 = 9;  // S2  cause reboot
const int motor2_In1 = 13;  // D7
const int motor2_In2 = 15;  // D8
// best not to set in pre-defined pin
const int motor1_PWM = 5;  //D1
const int motor2_PWM = 4;  //D2

// mcu input pins for motor's encoder
const int encoder1_C = 14;  //D5
// const int encoder1_D = 13;
// cannot use 3 and 1. Maybe GPIO1/3 is used for serial or other function by default...
const int encoder2_C = 12;  //D6
// const int encoder2_D = 9;
bool motor_dir[2];

// Variables for velocity, position
int encoder1_Count = 0;
int encoder2_Count = 0;
int encoder1_Count_b = 0;
int encoder2_Count_b = 0;

unsigned long prevTime = 0;      // To calculate elapsed time
const int encoder_slots = 20;     // Number of slots in encoder disk
const float wheelDiameter = 0.065;      // Wheel diameter in meter
const float wheelBase = 0.15;           // Distance between wheels in meter

bool usePid = false;
float motorSpeedMax;
PID pid_motorSpeed[2];

double linearVelocity = 0;
double angularVelocity = 0;
double linearVelocity_b = 0;
double angularVelocity_b = 0;

double posX = 0;  // robot's position X in meter
double posY = 0;
double heading = 90.0 * PI/180;  // robot's Heading angle in radian

// Interrupt service routines for encoder counting
float simBySpd_EncDt1 = 0;
float simBySpd_EncDt2 = 0;
const int encoder_dt = 1;  // sim: use 1 in physical project
void IRAM_ATTR encoder1_ISR() {
  // if (digitalRead(encoder1_D))
  // if (!digitalRead(motor1_In1)) {
  if (motor_dir[0]) {
    encoder1_Count = encoder1_Count + encoder_dt;
    // Serial.println("1++");
  } else {
    encoder1_Count = encoder1_Count - encoder_dt;
    // Serial.println("1--");
  }
}

void IRAM_ATTR encoder2_ISR() {
  // if (digitalRead(encoder2_D))
  // if (!digitalRead(motor2_In1)) {
  if (motor_dir[1]) {
    encoder2_Count = encoder2_Count + encoder_dt;
    // Serial.println("2++");
  } else {
    encoder2_Count = encoder2_Count - encoder_dt;
    // Serial.println("2--");
  }
}

// Function to set motor direction and speed
struct MotorPin {
  int in_1_L;
  int in_2_R;
  int in_pwm;
};
static MotorPin motorPin[2] = {
  {motor1_In1, motor1_In2, motor1_PWM},
  {motor2_In1, motor2_In2, motor2_PWM},
};
void setMotorSpeed(int motor, float speed) {
  // float simBySpd_EncDt = speed / 255 * 2;
  // if (motor == 1)
  //   simBySpd_EncDt1 = simBySpd_EncDt;
  // else
  //   simBySpd_EncDt2 = simBySpd_EncDt;

  int pwm = abs(speed);
  // pwm = (pwm == 255 ? 255 : pwm * 0.1);  // for simulation decay

  int mi = motor - 1;  // motor index

  if (speed == 0) {
    digitalWrite(motorPin[mi].in_1_L, LOW);
    digitalWrite(motorPin[mi].in_2_R, LOW);
  } else if (speed > 0) {
    digitalWrite(motorPin[mi].in_1_L, HIGH);
    digitalWrite(motorPin[mi].in_2_R, LOW);
    motor_dir[mi] = true;
  } else {
    digitalWrite(motorPin[mi].in_1_L, LOW);
    digitalWrite(motorPin[mi].in_2_R, HIGH);
    motor_dir[mi] = false;
  }

  if (!usePid) {
    if (mi == 0)
      pwm = 0.96 * pwm;  // motor 1 is faster then 2 when they got same input
    analogWrite(motorPin[mi].in_pwm, pwm);
    return;
  }

  const int pwmMax = 255;
  pid_motorSpeed[mi].target = motorSpeedMax * pwm / pwmMax;
}

int pwm_bf[2] = {
  0, 0
};
void appPidMotorSpeed() {
  if (!usePid)
    return;

  for (int mi=0; mi<2; mi++) {
    int pwm = pid_motorSpeed[mi].CalOutput_Pos();
    if (pwm_bf[mi] == pwm)
      return;
    pwm_bf[mi] = pwm;
    analogWrite(motorPin[mi].in_pwm, pwm);
    Serial.println(pid_motorSpeed[mi].getPlotString(mi+1));
    Serial.println(pid_motorSpeed[mi].getDataString_IE(mi+1));
  }
}

// Function to calculate velocity and position
void calculateOdometry() {
  // encoder1_Count += simBySpd_EncDt1;  // for sim
  // encoder2_Count += simBySpd_EncDt2;

  delay(200);  // must delay, otherwise the deltaTime could be zero
  // Calculate time elapsed
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0; // seconds
  prevTime = currentTime;

  int dt1 = encoder1_Count - encoder1_Count_b;
  int dt2 = encoder2_Count - encoder2_Count_b;
  encoder1_Count_b = encoder1_Count;
  encoder2_Count_b = encoder2_Count;

  // Calculate wheel speeds (m/s)
  static double distPerCount = (PI * wheelDiameter) / encoder_slots;
  pid_motorSpeed[0].actual = (dt1 * distPerCount) / deltaTime;
  pid_motorSpeed[1].actual = (dt2 * distPerCount) / deltaTime;

  // Calculate linear and angular velocity
  linearVelocity = (pid_motorSpeed[0].actual + pid_motorSpeed[1].actual) / 2;
  angularVelocity = (pid_motorSpeed[1].actual - pid_motorSpeed[0].actual) / wheelBase;

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
  // double heading_degP = heading * 180/PI;
  // Serial.printf("m1=%.3f, m2=%.3f, h=%.3f, vx=%.3f, vy=%.3f -- ", pid_motorSpeed[0].actual, pid_motorSpeed[1].actual, heading_degP, linearVelocity_x, linearVelocity_y);
  posX += linearVelocity_x * deltaTime;
  posY += linearVelocity_y * deltaTime;

  heading += angularVelocity * deltaTime;

  double angularVelocity_deg = angularVelocity * 180/PI;
  double heading_deg = heading * 180/PI;

  // Print speed, position
  Serial.printf("%.3fs -- Vel: lin=%.3f, ang=%.3f; Pos: x, y, h = %.3f, %.3f, %.3f\r\n", deltaTime, linearVelocity, angularVelocity_deg, posX, posY, heading_deg);

  Serial.println(pid_motorSpeed[0].getPlotString("1"));
  Serial.println(pid_motorSpeed[1].getPlotString("2"));

  if (!usePid) {
    return;
  }
  Serial.println(pid_motorSpeed[0].getDataString_IE("1"));
  Serial.println(pid_motorSpeed[1].getDataString_IE("2"));
}

bool bStopLoop = false;
void stopLoop(const char * msg=0) {
  bStopLoop = true;
  if (msg)
    Serial.println(msg);
  Serial.println("Stop Loop");
}

void setup() {
  Serial.begin(115200); // For debugging output
  Serial.println();
  Serial.println("---- setup ----");

  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  server.begin();

  // Motor pins setup
  pinMode(motor1_In1, OUTPUT);
  pinMode(motor1_In2, OUTPUT);
  pinMode(motor1_PWM, OUTPUT);
  pinMode(motor2_In1, OUTPUT);
  pinMode(motor2_In2, OUTPUT);
  pinMode(motor2_PWM, OUTPUT);

  // Encoder pins setup
  pinMode(encoder1_C, INPUT);    // INPUT_PULLUP
  // pinMode(encoder1_D, INPUT);
  pinMode(encoder2_C, INPUT);
  // pinMode(encoder2_D, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1_C), encoder1_ISR, FALLING);    //RISING
  attachInterrupt(digitalPinToInterrupt(encoder2_C), encoder2_ISR, FALLING);

  motorSpeedMax = 0.6;  // m/s, used as target speed.  full pwm -> 0.7
  pid_motorSpeed[0].setPID(700, 20, 20);
  pid_motorSpeed[1].setPID(700, 20, 20);
  pid_motorSpeed[0].setLimit(0, 255);
  pid_motorSpeed[1].setLimit(0, 255);

  Serial.println("---- setup finished ----");

  prevTime = millis();    // Initialize time

  Serial.println(pid_motorSpeed[0].getPlotString("1"));  // print a set of zero for plot
  Serial.println(pid_motorSpeed[1].getPlotString("2"));
  // stopLoop();
};

void DealClientData(WiFiClient *socket) {
  String cmdArgs = socket->readStringUntil('\n');
  char cmd[512];
  float speed1, speed2;
  int matched = sscanf(cmdArgs.c_str(), "%2s,%f,%f", cmd, &speed1, &speed2);  //%s needs specify the width...
  if (matched != 3) {
    Serial.printf("cmdArgs parse failed, cmdArgs=%s, matched=%d\n", cmdArgs.c_str(), matched);
    stopLoop();
    return;
  } else {
    Serial.printf("Received: %s,%.3f,%.3f\n", cmd, speed1, speed2);
    setMotorSpeed(1, speed1);
    setMotorSpeed(2, speed2);
  }
}

WiFiClient *socket = nullptr;
void MultiClientProcess() {
  if (!socket) {
    *socket = server.accept();
    if (socket->connected()) {
      Serial.println("Connect to client OK");
    } else {
      socket->stop();
      socket = nullptr;
      Serial.println("Connect to client failed");
    }
  }

  if (socket && socket->connected()) {
    // Serial.println("Connected to client");
  } else {
      socket->stop();
      socket = nullptr;
      Serial.println("Disconnected");
  }

  if (socket && socket->available()) {
    DealClientData(socket);
  }
}

void loop() {
  if (bStopLoop) {
    delay(10);
    return;
    // while(1);
  }

  WiFiClient client = server.accept();  // listen for incoming clients
  if (client) {  // if you get a client,
    Serial.println("New Client");
    while (client.connected()) {  // loop while the client's connected
      if (client.available()) { // read data available
        DealClientData(&client);
      }
      appPidMotorSpeed();
      calculateOdometry();
    }
    client.stop();  // if client is disconnected, stop client
    Serial.println("Client Disconnected");
  }

  appPidMotorSpeed();
  calculateOdometry();
}
