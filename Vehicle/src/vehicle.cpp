#include <Arduino.h>
// #include <string.h>
#include <ESP8266WiFi.h>

// WIFI AP
const char* ssid = "lch-EmSys_Vehicle";
const char* password = "lch12321";
const int port = 2020;

// mcu output pins to motor driver input
const int motor1_In1 = 16;
const int motor1_In2 = 5;
const int motor1_PWM = 4;
const int motor2_In1 = 0;
const int motor2_In2 = 2;
const int motor2_PWM = 14;

// mcu input pins for motor's encoder
const int encoder1_C = 12;
const int encoder1_D = 13;
const int encoder2_C = 15;  // cannot use 3 and 1. Maybe GPIO1 is used fro other function by default...
const int encoder2_D = 3;

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

WiFiServer server(2020);

bool bStopLoop = false;

void stopLoop(const char * msg=0) {
  bStopLoop = true;
  if (!msg)
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
  pinMode(motor2_In1, OUTPUT);
  pinMode(motor2_In2, OUTPUT);

  // Encoder pins setup
  pinMode(encoder1_C, INPUT);    // INPUT_PULLUP
  pinMode(encoder1_D, INPUT);
  pinMode(encoder2_C, INPUT);
  pinMode(encoder2_D, INPUT);
  // Serial.println("---- finished here ----");
  // return;
  attachInterrupt(digitalPinToInterrupt(encoder1_C), encoder1_ISR, FALLING);    //RISING
  attachInterrupt(digitalPinToInterrupt(encoder2_C), encoder2_ISR, FALLING);

  Serial.println("---- setup finished ----");

  prevTime = millis();    // Initialize time
};

void DealClientData(WiFiClient *socket) {
  // char cmdArgs[512];
  // client.readBytesUntil('\n', cmdArgs, 512);
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
    }
    client.stop();  // if client is disconnected, stop client
    Serial.println("Client Disconnected");
  }

  // ctrlSpeed();

  // calculateOdometry();
}
