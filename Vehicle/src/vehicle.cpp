#include <Arduino.h>
#include <WiFi.h>
#include "PID.h"
#include <WebServer.h>
#include <LittleFS.h>
#define FS LittleFS
void printPartition();

// #include <BluetoothSerial.h>
// BluetoothSerial SerialBT;  // used as remote serial port for printing
// #define Serial SerialBT

WiFiServer tcpSerServer(12321);
WiFiClient tcpSerCl;
#define Use_tcpSer
#define Serial tcpSerCl

// ref: https://github.com/espressif/arduino-esp32/tree/master/libraries/ArduinoOTA/examples
#include <ArduinoOTA.h>
bool ArduinoOTA_updating = false;

void setup();
void ArduinoOTASetup();

// web server
WebServer server(80);  // http port
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

void svSendSse(const String& message) {
  // Serial.println(message);
  if (sseClient.connected()) {
    // int t = esp_timer_get_time();
    sseClient.print("data: ");  //DO NOT forget data: ...
    sseClient.println(message);
    sseClient.println();  // end data
    // int tn = esp_timer_get_time();
    // Serial.println(tn-t);  // tcp msg: 10ms...
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

  // if (mi == 1) pwm *= 0.947;  //$ wheel align

  const int pwmMax = 255;
  pid_motorSpeed[mi].setpoint = motorSpeedMax * pwm / pwmMax;
}

String getPidPlotStr() {
  char tmp[512];
  sprintf(tmp, R"(<[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f])",
    pid_motorSpeed[0].setpoint,
    pid_motorSpeed[0].feedback,
    pid_motorSpeed[0].output,
    pid_motorSpeed[1].setpoint,
    pid_motorSpeed[1].feedback,
    pid_motorSpeed[1].output
  );
  return tmp;
}

void appPidMotorSpeed() {
  if (!usePid)
    return;

  static int pwm_bf[2] = {
    0, 0
  };
  int pwm[2];
  for (int mi=0; mi<2; mi++) {
    // pwm[mi] = pid_motorSpeed[mi].CalOutput_Inc();
    pwm[mi] = pid_motorSpeed[mi].CalOutput_Pos();
  }

  static PID pid_bf[2];  // just for debug
  static ulong pt_b = 0;
  bool mayPrint = false;
  ulong pt = millis();
  if (pt - pt_b > printCycTime) {
    pt_b = pt;
    mayPrint = true;
  }
  if (mayPrint && (pid_bf[0].isNotSame_Assign_Main(pid_motorSpeed[0]) || pid_bf[1].isNotSame_Assign_Main(pid_motorSpeed[1]))) {
    String msg = getPidPlotStr();
    svSendSse(msg);
  }

  for (int mi=0; mi<2; mi++) {
    if (pwm_bf[mi] == pwm[mi])
      continue;  // not return!!
    pwm_bf[mi] = pwm[mi];
    if (pid_motorSpeed[mi].setpoint == 0) {  // brake mode
      analogWrite(motorPin[mi].in_pwm, 255);
      continue;
    }
    analogWrite(motorPin[mi].in_pwm, pwm[mi]);
  }
}

// Function to calculate velocity and position
void calculateOdometry() {
  // encoder1_Count += simBySpd_EncDt1;  // for sim
  // encoder2_Count += simBySpd_EncDt2;

  delay(cycTime);  // must delay, otherwise the deltaTime could be zero
  // Calculate time elapsed
  static ulong prevTime = 0;
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
  // Serial.printf("%.3fs -- Vel: lin=%.3f, ang=%.3f; Pos: x, y, h = %.3f, %.3f, %.3f\r\n", deltaTime, linearVelocity, angularVelocity_deg, posX, posY, heading_deg);

  // data json to webpage
  char json[1024];
  sprintf(json, R"({"cyc_time":%.3f, "v_linear":%.3f, "v_angle":%.3f, "x":%.3f, "y":%.3f, "h":%.3f})", deltaTime, linearVelocity, angularVelocity_deg, posX, posY, heading_deg);

  // Serial.println(json);
  svSendSse(json);
}

bool bStopLoop = false;
void stopLoop(const char * msg=0) {
  bStopLoop = true;
  if (msg)
    Serial.println(msg);
  Serial.println("Stop Loop");
  digitalWrite(pin_ready, 0);
}

void serverOnSse() {
  Serial.println("serverOnSse");
  if (server.client()) {
    sseClient = server.client();
    sseClient.println("HTTP/1.1 200 OK");
    sseClient.println("Content-Type: text/event-stream");
    sseClient.println("Cache-Control: no-cache");
    sseClient.println("Connection: keep-alive");
    sseClient.println();  // end header
    Serial.println("serverOnSse OK");
  }
}

void serverOnPost() {
  String cmdArgs = server.arg("plain");

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

  server.send(200, "text/plain", "OK");  // send resp
}

void serverRegPathHandle(const String &path, String pathFile=String()) {
  if (pathFile.isEmpty())
    pathFile = path;
  server.on(path, HTTP_GET, [pathFile, path]() {
    File file = FS.open(pathFile, "r");
    if (!file) {
      Serial.printf("Failed to open file '%s' for '%s'\n", pathFile.c_str(), path.c_str());
      stopLoop();
      return;
    }
    auto suffix = pathFile.substring(pathFile.lastIndexOf(".") + 1);
    if (suffix == "html")
      server.streamFile(file, "text/html");
    else if (suffix == "css")
      server.streamFile(file, "text/css");
    else if (suffix == "js") {
      server.streamFile(file, "application/javascript");
    }
  });
}

void setup() {
  digitalWrite(pin_ready, 0);

#ifdef Use_tcpSer
#else
  Serial.begin(115200); // For debugging output
#endif

  Serial.println();
  Serial.println("---- setup ----");
  // printPartition();
  // return;

  WiFi.begin("R1216_2.4GHz", "r121612321");
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected as " + WiFi.localIP().toString());

#ifdef Use_tcpSer
  tcpSerServer.begin();
#endif

  if (!FS.begin()) {
    Serial.println("Failed to mount file system");
    stopLoop();
    return;
  }

  server.onNotFound([](){
    server.send(404, "text/plain", "---- Error Address! ----");
  });

  serverRegPathHandle("/", "/webpage.html");
  serverRegPathHandle("/style.css");
  serverRegPathHandle("/pidPlot.js");
  serverRegPathHandle("/script.js");

  server.on("/cmd", HTTP_POST, serverOnPost);
  server.on("/events", HTTP_GET, serverOnSse);

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

  motorSpeedMax = 0.5;  //$ m/s, used for setpoint speed.
  pid_motorSpeed[0].setPID(750, 75, 20);
  pid_motorSpeed[1].setPID(750, 75, 20);
  pid_motorSpeed[0].setLimit(0, 255);
  pid_motorSpeed[1].setLimit(0, 255);

  ArduinoOTASetup();
  ArduinoOTA.begin();
  // Serial.print("Free Heap: ");
  // Serial.println(ESP.getFreeHeap());

  Serial.println("---- setup finished ----");
  digitalWrite(pin_ready, 1);

  // stopLoop();
};

void ArduinoOTASetup()
{
  ArduinoOTA.setHostname("esp32-vehicle");
  ArduinoOTA
  .onStart([]() {
    ArduinoOTA_updating = true;
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nOTA End");
  })
  // for performance, we don't print progress
  // .onProgress([](unsigned int progress, unsigned int total) {
  //   static ulong tb = 0;
  //   ulong t = millis();
  //   if (t - tb < 500)
  //     return;
  //   Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  // })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
}

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
  if (tcpSerCl) {  // == tcpSerCl.connected()
  } else {
    tcpSerCl = tcpSerServer.available();  // try get new tcpSerCl
  }

  ArduinoOTA.handle();
  if (ArduinoOTA_updating) {
    return;
  }

  if (bStopLoop) {
    delay(10);
    return;
    // while(1);
  }

  server.handleClient();
  appPidMotorSpeed();
  calculateOdometry();
}

void printPartition() {
  Serial.println("---- printPartition ----");

  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);
  while (it != NULL) {
      const esp_partition_t* part = esp_partition_get(it);
      Serial.printf("Name: %s, Type: %d, SubType: %d, Address: 0x%X, Size: %dKB\n",
                    part->label, part->type, part->subtype, part->address, part->size / 1024);
      it = esp_partition_next(it);
  }
}