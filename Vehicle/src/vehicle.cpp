#include <Arduino.h>
#include <WiFi.h>
#include "PID.h"
#include <WebServer.h>
#include <LittleFS.h>

bool bBlynk = true;
bool bApMode = true;
void setup();

/* Fill in information from your Blynk Template here */
/* Read more: https://bit.ly/BlynkInject */
//#define BLYNK_TEMPLATE_ID           "TMPxxxxxx"
//#define BLYNK_TEMPLATE_NAME         "Device"
#define BLYNK_TEMPLATE_ID "TMPL6WPFtzYBT"
#define BLYNK_TEMPLATE_NAME "UTM EmSys A3"

#define BLYNK_FIRMWARE_VERSION        "0.1.0"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

// Uncomment your board, or configure a custom board in Settings.h
#define USE_ESP32_DEV_MODULE
//#define USE_ESP32C3_DEV_MODULE
//#define USE_ESP32S2_DEV_KIT
//#define USE_WROVER_BOARD
//#define USE_TTGO_T7
//#define USE_TTGO_T_OI

#include "BlynkEdgent.h"


// WIFI & server
WebServer myserver(80);  // http port
WiFiClient sseClient;  // keep life span

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

float deltaTime = 0;
float angularVelocity_deg = 0;
float heading_deg = 0;

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

String getRobotDataJson();
String getRobotDataShortString();
void setMotorSpeed_str(const String &cmdArgs);

// Ref: https://examples.blynk.cc/?board=ESP32&shield=ESP32%20WiFi&example=GettingStarted%2FPushData
// BLYNK_READ(V0)
void Blynk_pushData()
{
  static int tb = 0;
  int t = millis();
  if (t - tb > 500) {
    tb = t;
  } else {
    return;
  }

  // Blynk free plan only supports 5 widgets, while 1 label has limited width to display data
  String data = getRobotDataShortString() + ";"+ String(pid_motorSpeed[0].feedback) +","+ String(pid_motorSpeed[1].feedback);
  Blynk.virtualWrite(V0, data);
  Serial.print("Sent Data: ");
  Serial.println(data);
}
BLYNK_WRITE(V1)
{
  int pinData = param.asInt();
  if (pinData < 0) {
    setMotorSpeed_str("ms,-75,75");
    // Serial.println("LR");
  } else if (pinData > 0) {
    setMotorSpeed_str("ms,75,-75");
    // Serial.println("RR");
  } else {
    setMotorSpeed_str("ms,0,0");
  }
}
BLYNK_WRITE(V2)
{
  int pinData = param.asInt();
    Serial.println(pinData);

  if (pinData < 0) {
    setMotorSpeed_str("ms,-255,-255");
    // Serial.println("BW");
  } else if (pinData > 0) {
    setMotorSpeed_str("ms,255,255");
    // Serial.println("FW");
  } else {
    setMotorSpeed_str("ms,0,0");
  }
}

void sv_send_sse(const String& message) {
  // Serial.println(message);
  if (sseClient && sseClient.connected()) {
    sseClient.print("data: ");  //DO NOT forget data: ...
    sseClient.println(message);
    sseClient.println();  // end data
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
    // if (mi == 0)
    //   pwm = 0.96 * pwm;  // motor 1 is faster then 2 when they got same input
    analogWrite(motorPin[mi].in_pwm, pwm);
    return;
  }

  if (mi == 1) pwm *= 0.947;  //$ wheel align

  const int pwmMax = 255;
  pid_motorSpeed[mi].setpoint = motorSpeedMax * pwm / pwmMax;
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
  if (/* !bBlynk &&  */pt - pt_b > printCycTime) {
    pt_b = pt;
    mayPrint = true;
  }
  for (int mi=0; mi<2; mi++) {
    // int pwm = pid_motorSpeed[mi].CalOutput_Inc();
    int pwm = pid_motorSpeed[mi].CalOutput_Pos();

    if (mayPrint) {
      if (pid_bf[mi].isNotSame_Assign_Main(pid_motorSpeed[mi]))
        if (!bBlynk)
          sv_send_sse(pid_motorSpeed[mi].getJson(mi + 1));
        else
          Blynk_pushData();
        // Serial.println(pid_motorSpeed[mi].getPlotString(mi + 1));
      // if (pid_bf[mi].isNotSame_Assign_IE(pid_motorSpeed[mi]))
      //   Serial.println(pid_motorSpeed[mi].getDataString_IE(mi + 1));
    }

    if (pwm_bf[mi] == pwm)
      continue;  // not return!!
    pwm_bf[mi] = pwm;
    analogWrite(motorPin[mi].in_pwm, pwm);
  }
}

String getRobotDataShortString() {
  char json[1024];
  sprintf(json, "%dms v(%.1f,%.1f),p(%.1f,%.1f,%.1f)", int(deltaTime*1000), linearVelocity, angularVelocity_deg, posX, posY, heading_deg);
  return String(json);
}

String getRobotDataJson() {
  char json[1024];
  sprintf(json, R"({"cyc_time":%.3f, "v_linear":%.3f, "v_angle":%.3f, "x":%.3f, "y":%.3f, "h":%.3f})", deltaTime, linearVelocity, angularVelocity_deg, posX, posY, heading_deg);
  return String(json);
}

// Function to calculate velocity and position
void calculateOdometry() {
  // encoder1_Count += simBySpd_EncDt1;  // for sim
  // encoder2_Count += simBySpd_EncDt2;

  delay(cycTime);  // must delay, otherwise the deltaTime could be zero
  // Calculate time elapsed
  static unsigned long prevTime = 0;
  unsigned long currentTime = millis();
  deltaTime = (currentTime - prevTime) / 1000.0; // seconds
  prevTime = currentTime;

  int dt1 = encoder1_Count - encoder1_Count_b;
  int dt2 = encoder2_Count - encoder2_Count_b;
  encoder1_Count_b = encoder1_Count;
  encoder2_Count_b = encoder2_Count;

  // Calculate wheel speeds (m/s)
  static double distPerCount = PI * wheelDiameter / encoder_slots / gearRate;
  float wlv_1 = dt1 * distPerCount / deltaTime;  // wheel linear velocity
  float wlv_2 = dt2 * distPerCount / deltaTime;
  pid_motorSpeed[0].feedback = abs(wlv_1);
  pid_motorSpeed[1].feedback = abs(wlv_2);

  // Calculate linear and angular velocity
  linearVelocity = (wlv_1 + wlv_2) / 2;
  angularVelocity = (wlv_2 - wlv_1) / wheelBase;

  if (linearVelocity_b == linearVelocity && angularVelocity_b == angularVelocity) {
    return;
  }
  linearVelocity_b = linearVelocity;
  angularVelocity_b = angularVelocity;

  // if (0 == linearVelocity && 0 == angularVelocity) {
  //   return;
  // }

  // Update robot's position
  double linearVelocity_x = linearVelocity * cos(heading);  // cos/sin() is in radian!
  double linearVelocity_y = linearVelocity * sin(heading);
  // Serial.printf("vx=%.3f, vy=%.3f -- ", , linearVelocity_x, linearVelocity_y);
  posX += linearVelocity_x * deltaTime;
  posY += linearVelocity_y * deltaTime;

  heading += angularVelocity * deltaTime;

  angularVelocity_deg = angularVelocity * 180/PI;
  heading_deg = heading * 180/PI;

  // Print speed, position
  static int pt_b = 0;
  int pt = millis();
  if (pt - pt_b < printCycTime)
    return;
  pt_b = pt;

  // Serial.printf("enc1=%d, enc2=%d, enc1/enc2=%f\n", encoder1_Count, encoder2_Count, float(encoder1_Count)/encoder2_Count);  // Not for wheel align
  // Serial.printf("%.3fs -- Vel: lin=%.3f, ang=%.3f; Pos: x, y, h = %.3f, %.3f, %.3f\r\n", deltaTime, linearVelocity, angularVelocity_deg, posX, posY, heading_deg);

  if (!bBlynk)
    sv_send_sse(getRobotDataJson());
  else {
    Blynk_pushData();
  }
}

bool bStopLoop = false;
void stopLoop(const char * msg=0) {
  bStopLoop = true;
  if (msg)
    Serial.println(msg);
  Serial.println("Stop Loop");
}

void serverOnSse() {
  Serial.println("serverOnSse");
  if (myserver.client()) {
    sseClient = myserver.client();
    sseClient.println("HTTP/1.1 200 OK");
    sseClient.println("Content-Type: text/event-stream");
    sseClient.println("Cache-Control: no-cache");
    sseClient.println("Connection: keep-alive");
    sseClient.println();  // end header
    Serial.println("serverOnSse OK");
  }
}

void setMotorSpeed_str(const String &cmdArgs) {
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

void serverOnPost() {
  String cmdArgs = myserver.arg("plain");
  setMotorSpeed_str(cmdArgs);
  myserver.send(200, "text/plain", "OK");  // send resp
}

void setup() {
  Serial.begin(115200); // For debugging output
  Serial.println();
  Serial.println("---- setup ----");

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

  motorSpeedMax = 0.5;  //$ m/s, used for setpoint speed.
  pid_motorSpeed[0].setPID(2200, 75, 20);
  pid_motorSpeed[1].setPID(2200, 75, 20);
  pid_motorSpeed[0].setLimit(0, 255);
  pid_motorSpeed[1].setLimit(0, 255);

  if (bBlynk) {
    BlynkEdgent.begin();
    Serial.println("---- setup BlynkEdgent finished ----");
    return;
  }

  if (bApMode) {
    WiFi.softAP("lch-EmSys_Vehicle", "lch12321");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    WiFi.begin("R1216_2.4GHz", "r121612321");
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
    Serial.println("\nConnected as " + WiFi.localIP().toString());
  }

  if (!LittleFS.begin()) {
    Serial.println("Failed to mount file system");
    stopLoop();
    return;
  }

  myserver.onNotFound([](){
    myserver.send(404, "text/plain", "Error Address");
  });

  myserver.on("/", HTTP_GET, []() {
    Serial.println("html started");
    File file = LittleFS.open("/webpage.html", "r");
    if (!file) {
      Serial.println("Failed to open file -html");
      stopLoop();
      return;
    }
    myserver.streamFile(file, "text/html");
  });

  myserver.on("/style.css", HTTP_GET, []() {
    Serial.println("css started");
    File file = LittleFS.open("/style.css", "r");
    if (!file) {
      Serial.println("Failed to open file -style");
      stopLoop();
      return;
    }
    myserver.streamFile(file, "text/css");
  });

  myserver.on("/script.js", HTTP_GET, []() {
    Serial.println("script started");
    File file = LittleFS.open("/script.js", "r");
    if (!file) {
      Serial.println("Failed to open file -script");
      stopLoop();
      return;
    }
    myserver.streamFile(file, "application/javascript");
  });

  myserver.on("/cmd", HTTP_POST, serverOnPost);
  myserver.on("/events", HTTP_GET, serverOnSse);

  myserver.begin();

  Serial.println("---- setup finished ----");
  // stopLoop();
};

void serverConnectState() {
  static bool cnt_bf = false;
  bool cnt = myserver.client().connected();
  if (cnt_bf != cnt) {
    cnt_bf = cnt;
    if (cnt) {
      Serial.println("New Client");
      digitalWrite(pin_ready, 1);
      for (int i=0; i<2; i++) {
        Serial.println(pid_motorSpeed[i].getPlotString(i + 1));  // print a set of initial zeros for plot
      }
    } else {
      digitalWrite(pin_ready, 0);
      // client.stop();  // if client is disconnected, stop client
      Serial.println("Client Disconnected");
    }
  }
}

void loop() {
  if (bStopLoop) {
    delay(10);
    return;
    // while(1);
  }

  if (bBlynk) {
    BlynkEdgent.run();
    static bool initPushed = false;
    if (!initPushed && Blynk.connected()) {
      Blynk_pushData(); Serial.println("initial push");
      initPushed = true;
    }
  } else {
    myserver.handleClient();
    serverConnectState();
  }

  appPidMotorSpeed();
  calculateOdometry();
}
