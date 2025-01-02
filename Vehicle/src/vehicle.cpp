#include <Arduino.h>
// #include <string.h>
#include <WiFi.h>
#include "PID.h"
// #include "PID_v1.h"
#include <WebServer.h>
#include <LittleFS.h>

// WIFI AP & tcp server
const char* ssid = "lch-EmSys_Vehicle";
const char* password = "lch12321";
const int port = 80;
WebServer server(port);

const int pin_ready = 2;

// mcu output pins to motor driver input
const int motor1_In1 = 17;
const int motor1_In2 = 16;
const int motor1_PWM = 4;
const int motor_STB = 5;  // standby
const int motor2_In1 = 18;
const int motor2_In2 = 19;
const int motor2_PWM = 21;

// mcu input pins for motor's encoder
const int encoder1_C = 25;
const int encoder1_D = 26;
const int encoder2_C = 32;
const int encoder2_D = 33;
bool motor_dir[2];

// Variables for velocity, position
int encoder1_Count = 0;
int encoder2_Count = 0;
int encoder1_Count_b = 0;
int encoder2_Count_b = 0;

unsigned long prevTime = 0;      // To calculate elapsed time
const int encoder_slots = 13;     // Number of slots in encoder disk
const int gearRate = 90;
const float wheelDiameter = 0.065;      // Wheel diameter in meter
const float wheelBase = 0.155;           // Distance between wheels in meter

bool usePid = true;  //$
const int pi_DebugPID = 23;
float motorSpeedMax;
const int cycTime = 10;
const int printCycTime = 100;
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

void IRAM_ATTR DebugPID() {
  // Serial.println("DebugPID");
  pid_motorSpeed[0].printVars = true;
  pid_motorSpeed[1].printVars = true;
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
    // if (mi == 0)
    //   pwm = 0.96 * pwm;  // motor 1 is faster then 2 when they got same input
    analogWrite(motorPin[mi].in_pwm, pwm);
    return;
  }

  if (mi == 1) pwm *= 0.947;  //$ wheel align

  const int pwmMax = 255;
  pid_motorSpeed[mi].target = motorSpeedMax * pwm / pwmMax;
}


void appPidMotorSpeed() {
  if (!usePid)
    return;

  static int pwm_bf[2] = {
    0, 0
  };

  static PID pid_bf[2];  // just for debug
  static int pt_b = 0;
  bool mayPrint = false;
  int pt = millis();
  if (pt - pt_b > printCycTime) {
    pt_b = pt;
    mayPrint = true;
  }
  for (int mi=0; mi<2; mi++) {
    // int pwm = pid_motorSpeed[mi].CalOutput_Inc();
    int pwm = pid_motorSpeed[mi].CalOutput_Pos();

    if (mayPrint) {
      if (pid_bf[mi].isNotSame_Assign_Main(pid_motorSpeed[mi]))
        Serial.println(pid_motorSpeed[mi].getPlotString(mi + 1));
      if (pid_bf[mi].isNotSame_Assign_IE(pid_motorSpeed[mi]))
        Serial.println(pid_motorSpeed[mi].getDataString_IE(mi + 1));
    }

    if (pwm_bf[mi] == pwm)
      continue;  // not return!!
    pwm_bf[mi] = pwm;
    analogWrite(motorPin[mi].in_pwm, pwm);
  }
}

// Function to calculate velocity and position
void calculateOdometry() {
  // encoder1_Count += simBySpd_EncDt1;  // for sim
  // encoder2_Count += simBySpd_EncDt2;

  delay(cycTime);  // must delay, otherwise the deltaTime could be zero
  // Calculate time elapsed
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0; // seconds
  prevTime = currentTime;

  int dt1 = encoder1_Count - encoder1_Count_b;
  int dt2 = encoder2_Count - encoder2_Count_b;
  encoder1_Count_b = encoder1_Count;
  encoder2_Count_b = encoder2_Count;

  // Calculate wheel speeds (m/s)
  static double distPerCount = PI * wheelDiameter / encoder_slots / gearRate;
  float wlv_1 = dt1 * distPerCount / deltaTime;  // wheel linear velocity
  float wlv_2 = dt2 * distPerCount / deltaTime;
  pid_motorSpeed[0].actual = abs(wlv_1);
  pid_motorSpeed[1].actual = abs(wlv_2);

  // Calculate linear and angular velocity
  linearVelocity = (wlv_1 + wlv_2) / 2;
  angularVelocity = (wlv_2 - wlv_1) / wheelBase;

  if (linearVelocity_b == linearVelocity && angularVelocity_b == angularVelocity) {
    return;
  }
  linearVelocity_b = linearVelocity;
  angularVelocity_b = angularVelocity;

  if (0 == linearVelocity && 0 == angularVelocity) {
    return;
  }

  // Update robot's position
  double linearVelocity_x = linearVelocity * cos(heading);  // cos/sin() is in radian!
  double linearVelocity_y = linearVelocity * sin(heading);
  // Serial.printf("vx=%.3f, vy=%.3f -- ", , linearVelocity_x, linearVelocity_y);
  posX += linearVelocity_x * deltaTime;
  posY += linearVelocity_y * deltaTime;

  heading += angularVelocity * deltaTime;

  double angularVelocity_deg = angularVelocity * 180/PI;
  double heading_deg = heading * 180/PI;

  // Print speed, position
  static int pt_b = 0;
  int pt = millis();
  if (pt - pt_b < printCycTime)
    return;
  pt_b = pt;

  // Serial.printf("enc1=%d, enc2=%d, enc1/enc2=%f\n", encoder1_Count, encoder2_Count, float(encoder1_Count)/encoder2_Count);  // Not for wheel align
  // data
  Serial.printf("%.3fs -- Vel: lin=%.3f, ang=%.3f; Pos: x, y, h = %.3f, %.3f, %.3f\r\n", deltaTime, linearVelocity, angularVelocity_deg, posX, posY, heading_deg);
}

bool bStopLoop = false;
void stopLoop(const char * msg=0) {
  bStopLoop = true;
  if (msg)
    Serial.println(msg);
  Serial.println("Stop Loop");
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void serverOnPost() {
  if (server.uri() != "/cmd") {
    return;
  }
  if (!server.hasArg("ms")) {
    return;
  }

  String cmdArgs = server.arg("ms");
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

void setup() {
  Serial.begin(115200); // For debugging output
  Serial.println();
  Serial.println("---- setup ----");

  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  if (!LittleFS.begin()) {
    Serial.println("Failed to mount file system");
    stopLoop();
    return;
  }
  File file = LittleFS.open("/file.txt", "r");
  if (!file) {
    Serial.println("Failed to open file");
    stopLoop();
    return;
  }
  Serial.println(file.readString());

  server.onNotFound(handleNotFound);
  server.on("/", HTTP_POST, serverOnPost);
  // server.on("/", HTTP_GET, serverOnPost);
  server.on("/wheel", [](){
    String msg = pid_motorSpeed[0].getPlotString("1") + "\n" + pid_motorSpeed[1].getPlotString("2");
    server.send(200, "text/plain", msg);
    // server.send(200, "text/plain", pid_motorSpeed[1].getPlotString("2"));  // can only one msg be sent
  });
  server.on("/robot", [](){
    server.send(200, "text/plain", pid_motorSpeed[1].getPlotString("1"));
  });
  server.begin();

  pinMode(pin_ready, OUTPUT);

  // Motor pins setup
  pinMode(motor1_In1, OUTPUT);
  pinMode(motor1_In2, OUTPUT);
  pinMode(motor1_PWM, OUTPUT);
  pinMode(motor_STB, OUTPUT);
  pinMode(motor2_In1, OUTPUT);
  pinMode(motor2_In2, OUTPUT);
  pinMode(motor2_PWM, OUTPUT);
  digitalWrite(motor_STB, 1);  // always standby

  // Encoder pins setup
  pinMode(encoder1_C, INPUT);    // INPUT_PULLUP
  // pinMode(encoder1_D, INPUT);
  pinMode(encoder2_C, INPUT);
  // pinMode(encoder2_D, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1_C), encoder1_ISR, FALLING);    //RISING
  attachInterrupt(digitalPinToInterrupt(encoder2_C), encoder2_ISR, FALLING);

  pinMode(pi_DebugPID, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(pi_DebugPID), DebugPID, RISING);

  motorSpeedMax = 0.5;  //$ m/s, used for target speed.
  pid_motorSpeed[0].setPID(2200, 75, 20);
  pid_motorSpeed[1].setPID(2200, 75, 20);
  pid_motorSpeed[0].setLimit(0, 255);
  pid_motorSpeed[1].setLimit(0, 255);

  Serial.println("---- setup finished ----");

  prevTime = millis();    // Initialize time
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

void loop() {
  if (bStopLoop) {
    delay(10);
    return;
    // while(1);
  }

  server.handleClient();
  appPidMotorSpeed();
  calculateOdometry();

  static bool cnt_bf = false;
  bool cnt = server.client().connected();
  if (cnt_bf != cnt) {
    cnt_bf = cnt;
    if (cnt) {
      Serial.println("New Client");
      digitalWrite(pin_ready, 1);
      for (int i=0; i<2; i++) {
        Serial.println(pid_motorSpeed[i].getPlotString(i + 1));  // print a set of initial zeros for plot
        Serial.println(pid_motorSpeed[i].getDataString_IE(i + 1));
      }
    } else {
      digitalWrite(pin_ready, 0);
      // client.stop();  // if client is disconnected, stop client
      Serial.println("Client Disconnected");
    }
  }
}
