#include <Arduino.h>
#include <WiFi.h>
#include "PID.h"
#include <LittleFS.h>
#define MyFS LittleFS
void printPartition();
#include <Preferences.h>
Preferences nvs;

#include "json.hpp"
using json = nlohmann::ordered_json;

// #include <WebServer.h>
// web server
// WebServer server(80);  // http port
// WiFiClient sseClient;  // keep life span

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
AsyncWebServer server(80);
AsyncEventSource events("/events");
// AsyncWebSocket ws("/ws");

// #include <BluetoothSerial.h>
// BluetoothSerial SerialBT;  // used as remote serial port for printing
// #define xSerial SerialBT

// WiFiServer tcpSerServer(12321);
// WiFiClient tcpSerCl;
// #define Use_tcpSer
// #define xSerial tcpSerCl

// #include <SseSer.h>
// SseSer sseSer(&sseClient);
// #define xSerial sseSer

#include <SseSerAsyn.h>
SseSerAsyn sseSer(&events);
#define xSerial sseSer

// ref: https://github.com/espressif/arduino-esp32/tree/master/libraries/ArduinoOTA/examples
#include <ArduinoOTA.h>
bool ArduinoOTA_updating = false;

void setup();
void ArduinoOTASetup();

const int pin_ready = 2;

// mcu output pins to motor driver input
const int motor1_In1 = 19;
const int motor1_In2 = 18;
const int motor1_PWM = 21;
const int motor_STB = 5;  // standby
const int motor2_In1 = 16;
const int motor2_In2 = 17;
const int motor2_PWM = 4;

// mcu input pins for motor's encoder
const int encoder1_C = 32;
const int encoder1_D = 33;
const int encoder2_C = 25;
const int encoder2_D = 26;
// bool motor_dir[2];
const int pinBattery = 35;

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
float deltaTime = 0;
const int printCycTime = 100;
PID pid_motorSpeed[2];
json jCfgRoot;
// json *jCfgPid[2];
// std::reference_wrapper <json> jCfgPid[2];  array requires a default constructor
std::vector<std::reference_wrapper<json>> jCfgPid;

double linearVelocity = 0;
double angularVelocity = 0;
float angularVelocity_deg = 0;
double linearVelocity_b = 0;
double angularVelocity_b = 0;

double posX = 0;  // robot's position X in meter
double posY = 0;
double heading = 90.0 * PI/180;  // robot's Heading angle in radian
float heading_deg = 90;

// Interrupt service routines for encoder counting
float simBySpd_EncDt1 = 0;
float simBySpd_EncDt2 = 0;
const int encoder_dt = 1;  // sim: use 1 in physical project
void IRAM_ATTR encoder1_ISR() {
  if (digitalRead(encoder1_D)) {
  // if (motor_dir[0]) {
    encoder1_Count = encoder1_Count + encoder_dt;
  } else {
    encoder1_Count = encoder1_Count - encoder_dt;
  }
}

void IRAM_ATTR encoder2_ISR() {
  if (!digitalRead(encoder2_D)) {
  // if (motor_dir[1]) {
    encoder2_Count = encoder2_Count + encoder_dt;
  } else {
    encoder2_Count = encoder2_Count - encoder_dt;
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
    // motor_dir[mi] = true;
  } else {
    digitalWrite(motorPin[mi].in_1_L, LOW);
    digitalWrite(motorPin[mi].in_2_R, HIGH);
    // motor_dir[mi] = false;
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

  static float pwm_bf[2] = {
    0, 0
  };
  float pwm[2];
  for (int mi=0; mi<2; mi++) {
    // pwm[mi] = pid_motorSpeed[mi].CalOutput_Inc();
    pwm[mi] = pid_motorSpeed[mi].CalOutput_Inc();
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
    xSerial.println(getPidPlotStr());
  }
  // if (mayPrint && (pid_bf[0].isNotSame_Assign_IE(pid_motorSpeed[0]) || pid_bf[1].isNotSame_Assign_IE(pid_motorSpeed[1]))) {
  //   Serial.println(pid_bf[0].getDataString_IE(1) + "--" + pid_bf[1].getDataString_IE(2) );
  // }

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

String getBasicData() {
  char json[256];
  sprintf(json, R"({"cyc_time":%.3f, "v_linear":%.3f, "v_angle":%.3f, "x":%.3f, "y":%.3f, "h":%.3f})", deltaTime, linearVelocity, angularVelocity_deg, posX, posY, heading_deg);
  return json;
}

// Function to calculate velocity and position
void calculateOdometry() {
  // encoder1_Count += simBySpd_EncDt1;  // for sim
  // encoder2_Count += simBySpd_EncDt2;

  delay(cycTime);  // must delay, otherwise the deltaTime could be zero
  // Calculate time elapsed
  static ulong prevTime = 0;
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

  if (0 == linearVelocity && 0 == angularVelocity) {
    return;
  }

  // Update robot's position
  double linearVelocity_x = linearVelocity * cos(heading);  // cos/sin() is in radian!
  double linearVelocity_y = linearVelocity * sin(heading);
  // xSerial.printf("vx=%.3f, vy=%.3f -- ", , linearVelocity_x, linearVelocity_y);
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
  xSerial.println(getBasicData());
}

bool bStopLoop = false;
void stopLoop(const char * msg=0) {
  bStopLoop = true;
  if (msg)
    xSerial.println(msg);
  xSerial.println("Stop Loop");
  digitalWrite(pin_ready, 0);
}

void listFiles() {
  File root = LittleFS.open("/");
  if (!root) {
      Serial.println("Failed to open root directory");
      return;
  }
  if (!root.isDirectory()) {
      Serial.println("Root is not a directory");
      return;
  }

  Serial.println("Listing all files:");
  File file = root.openNextFile();
  while (file) {
      Serial.printf("File: %s, Size: %d bytes\n", file.name(), file.size());
      file = root.openNextFile();
  }
}

String LoadPidConfig() {
  xSerial.printf("Free heap memory: %d bytes\n", esp_get_free_heap_size());
  // listFiles();

  auto fileDef = MyFS.open("/configDef.json", "r");
  if (fileDef) {
    String sFile;
    while (fileDef.available()) { sFile += char(fileDef.read()); }
    jCfgRoot = json::parse(sFile.c_str(), nullptr, false, true);
    // xSerial.printf("jCfgRoot of def: %s\n", jCfgRoot.dump().c_str());
  } else {
    xSerial.printf("Failed to open file %s for reading\n", fileDef.path());
    return "";
  }

  char buf[512];
  auto size = nvs.getBytes("config.json", buf, sizeof(buf));
  if (size) {
    buf[size] = 0;
    xSerial.printf("nvs: config.json found: %s\n", buf);
    jCfgRoot = json::parse(buf, nullptr, false, true);
  }
  xSerial.printf("jCfgRoot: %s\n", jCfgRoot.dump().c_str());

  jCfgPid.clear();
  for (int i=0; i<2; i++) {
    auto &jPid = jCfgRoot["pid"]["velLoop_inc"][i];
    jCfgPid.push_back(std::ref(jPid));
    pid_motorSpeed[i].setPID(jPid["p"], jPid["i"], jPid["d"]);
  }

  char tmp[256];
  sprintf(tmp, R"(N{"pid1_p":%f, "pid1_i":%f, "pid1_d":%f, "pid2_p":%f, "pid2_i":%f, "pid2_d":%f})",
    pid_motorSpeed[0].kp, pid_motorSpeed[0].ki, pid_motorSpeed[0].kd,
    pid_motorSpeed[1].kp, pid_motorSpeed[1].ki, pid_motorSpeed[1].kd);
  xSerial.print(tmp);
  return tmp;
}

void UpdataPidConfig(float v[6]) {
  if (v[0] == 9999) {
    MyFS.remove("/config.json");
    return;
  }

  for (int i=0; i<2; i++) {
    jCfgPid[i].get()["p"] = v[i*3];
    jCfgPid[i].get()["i"] = v[i*3+1];
    jCfgPid[i].get()["d"] = v[i*3+2];
    pid_motorSpeed[i].setPID(v[i*3], v[i*3+1], v[i*3+2]);
  }
  // xSerial.printf("jCfgRoot: %s\n", jCfgRoot.dump().c_str());

  std::string sFile = jCfgRoot.dump();
  auto size = nvs.putBytes("config.json", sFile.c_str(), sFile.length());
  if (size < sFile.length()) {
    xSerial.printf("nvs: config.json write error\n");
  }
}

void serverOnPost(AsyncWebServerRequest *request) {
  if (request->hasParam("ms", true)) {
    String args = request->getParam("ms", true)->value();
    float speed1, speed2;
    int matched = sscanf(args.c_str(), "%f,%f", &speed1, &speed2);
    if (matched != 2) {
      xSerial.printf("ms: args parse failed, args=%s, matched=%d\n", args.c_str(), matched);
      stopLoop();
      return;
    } else {
      xSerial.printf("Received: ms=%.3f,%.3f\n", speed1, speed2);
      setMotorSpeed(1, speed1);
      setMotorSpeed(2, speed2);
    }
  } else if (request->hasParam("pid", true)) {
    String args = request->getParam("pid", true)->value();
    float v[6];
    int matched = sscanf(args.c_str(), "%f,%f,%f;%f,%f,%f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]);
    if (matched != 6) {
      xSerial.printf("pid: args parse failed, args=%s, matched=%d\n", args.c_str(), matched);
      stopLoop();
      return;
    } else {
      xSerial.printf("Received: pid=%s\n", args.c_str());
      UpdataPidConfig(v);
    }
  } else {
    request->send(400, "text/plain", "ESP32: Bad Request - Missing 'ms' parameter");
    return;
  }

  request->send(200, "text/plain", "");  // send resp
}

bool sseAfConnect = false;

void setup() {
  digitalWrite(pin_ready, 0);

  Serial.begin(115200); // For debugging output
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
  WiFi.setSleep(false); // Improve real-time performance

#ifdef Use_tcpSer
  tcpSerServer.begin();
#endif

  if (!MyFS.begin()) {
    xSerial.println("Failed to mount file system");
    stopLoop();
    return;
  }
  if (!nvs.begin("config.json", false)) {
    xSerial.println("Failed to mount NVS");
    stopLoop();
    return;
  }

  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404, "text/plain", "---- Error Address! ----");
  });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(MyFS, "/webpage.html", "text/html");
  });
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(MyFS, "/style.css", "text/css");
  });
  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(MyFS, "/script.js", "application/javascript");
  });
  server.on("/pidPlot.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(MyFS, "/pidPlot.js", "application/javascript");
  });
  server.on("/joy.min.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(MyFS, "/joy.min.js", "application/javascript");
  });

  // server.on("/??", HTTP_GET, [](AsyncWebServerRequest *request) {
  //   request->send(200, "text/plain", "ESP32: GET Request Received");
  // });
  server.on("/cmd", HTTP_POST, serverOnPost);

  // server.on("/events", HTTP_GET, serverOnSse);
  server.addHandler(&events);
  events.onConnect([/* uiStr */](AsyncEventSourceClient *client){
    // events.send("?");  // events cannot send message in events.onConnect() procedure, other wise crash
    sseAfConnect = true;
  });

  // ws.onEvent(onWebSocketEvent);
  // server.addHandler(&ws);

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
  pinMode(encoder1_D, INPUT);
  pinMode(encoder2_C, INPUT);
  pinMode(encoder2_D, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1_C), encoder1_ISR, FALLING);    //RISING
  attachInterrupt(digitalPinToInterrupt(encoder2_C), encoder2_ISR, FALLING);

  pinMode(pinBattery, INPUT);

  pinMode(pi_DebugPID, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(pi_DebugPID), DebugPID, RISING);

  motorSpeedMax = 0.5;  //$ m/s, used for setpoint speed.

  ArduinoOTASetup();
  ArduinoOTA.begin();
  // xSerial.print("Free Heap: ");
  // xSerial.println(ESP.getFreeHeap());

  xSerial.println("---- setup finished ----");
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
    xSerial.println("Start updating " + type);
  })
  .onEnd([]() {
    xSerial.println("\nOTA End");
  })
  // for performance, we don't print progress
  // .onProgress([](unsigned int progress, unsigned int total) {
  //   static ulong tb = 0;
  //   ulong t = millis();
  //   if (t - tb < 500)
  //     return;
  //   xSerial.printf("Progress: %u%%\r", (progress / (total / 100)));
  // })
  .onError([](ota_error_t error) {
    xSerial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      xSerial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      xSerial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      xSerial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      xSerial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      xSerial.println("End Failed");
    }
  });
}

void BatteryCheck() {
  static ulong t_batteryB = 0;
  ulong t_battery = millis();
  if (t_battery - t_batteryB > 5000) {
    t_batteryB = t_battery;
    const float maxV = 3.7 * 2 / 11.0;  // 2 3.7V battery; 11.0: TB6612 model voltage divider
    float curV = analogRead(pinBattery) / 4095.0 * 3.3;
    int percentage = curV / maxV * 100;
    if (percentage < 20) {
      xSerial.println("Battery Low");
    }
  }
}

void loop() {
  BatteryCheck();

#ifdef Use_tcpSer
  if (tcpSerCl) {  // == tcpSerCl.connected()
  } else {
    tcpSerCl = tcpSerServer.available();  // try get new tcpSerCl
  }
#endif
  // ws.cleanupClients(); // Keep WebSocket clients alive

  ArduinoOTA.handle();
  if (ArduinoOTA_updating) {
    return;
  }

  if (bStopLoop) {
    delay(10);
    return;
    // while(1);
  }

  if (sseAfConnect) {
    sseAfConnect = false;
    LoadPidConfig();
    xSerial.println(getBasicData());
    xSerial.println(getPidPlotStr());
  }

  appPidMotorSpeed();
  calculateOdometry();
}

void printPartition() {
  xSerial.println("---- printPartition ----");

  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);
  while (it != NULL) {
      const esp_partition_t* part = esp_partition_get(it);
      xSerial.printf("Name: %s, Type: %d, SubType: %d, Address: 0x%X, Size: %dKB\n",
                    part->label, part->type, part->subtype, part->address, part->size / 1024);
      it = esp_partition_next(it);
  }
}