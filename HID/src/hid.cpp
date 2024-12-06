#include <Arduino.h>
// #include <string.h>
#include <WIFI.h>

const char* ssid = "R1216_2.4GHz";
const char* password = "r121612321";
const char* host = "192.168.1.4";

// const char* ssid = "LCH-DELL";
// const char* password = "lch12345678";
// const char* host = "192.168.137.1";

const int port = 2020;

int NotLoop = 0;
WiFiClient client;

// mcu input pins for joystick
const int joystick_Vert_Pin = 36;
const int joystick_Horz_Pin = 39;
// zero-offset
int joystick_vert_zo = INT_MAX;
int joystick_horz_zo = INT_MAX;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("---- setup ----");

  // We start by connecting to a WiFi network
  Serial.printf("Connecting to %s\r\n", ssid);
  WiFi.begin(ssid, password);

  int connect_time = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    connect_time += 500;
    if (connect_time > 10000) {
      Serial.println("WiFi connect failed, timeout");
      NotLoop++;
      return;
    }
  }

  Serial.println();
  Serial.printf("WiFi connected with ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.print("connecting to robot server: ");
  Serial.println(host);

  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    ++NotLoop;
    return;
  }

  Serial.println("server connected");
  Serial.println();

  joystick_vert_zo = analogRead(joystick_Vert_Pin);
  joystick_horz_zo = analogRead(joystick_Horz_Pin);
}

void stopLoop(const char * msg) {
  NotLoop++;
  Serial.println(msg);
  Serial.println("closing connection");
  client.stop();
}

void setMotorSpeed(float speed1, float speed2)
{
  client.printf("ms,%.3f,%.3f\n", speed1, speed2);
  // client.flush();
  return;

  char resp;
  int readLen = client.readBytesUntil('\n', &resp, 1);
  if (readLen == 0) {
    stopLoop("Getting svr's resp timeout");
    return;
  }
  if (resp != '0') {
    stopLoop("Svr's resp err");
    return;
  }
}

int vert_b = 0;
int horz_b = 0;

// for measuring vibratingValue
int vMin = 0;
int vMax = 0;
int hMin = 0;
int hMax = 0;

void ctrlSpeed() {
  delay(500);
  int vert = analogRead(joystick_Vert_Pin);
  int horz = analogRead(joystick_Horz_Pin);
  vert = vert - joystick_vert_zo;
  horz = horz - joystick_horz_zo;

  // if (vert < vMin) vMin = vert;
  // if (vert > vMax) vMax = vert;
  // if (horz < hMin) hMin = horz;
  // if (horz > hMax) hMax = horz;

  const int vibratingValue = 100;
  if (abs(vert_b - vert) <= vibratingValue && abs(horz_b - horz) <= vibratingValue)
    return;
  vert_b = vert;
  horz_b = horz;

  // boundary deal
  if (abs(vert - 0)     <= vibratingValue * 2) vert = 0;
  if (abs(vert - 2048)  <= vibratingValue * 2) vert = 2048;
  if (abs(vert - -2048) <= vibratingValue * 2) vert = -2048;  // use -2048 instead of -2047 to avoid zero mapping to +-1 below
  if (abs(horz - 0)     <= vibratingValue * 2) horz = 0;
  if (abs(horz - 2048)  <= vibratingValue * 2) horz = 2048;
  if (abs(horz - -2048) <= vibratingValue * 2) horz = -2048;

  // client.printf("ms\t%d\t%d\t%d\t%d\t%d\t%d\n", vert, horz, vMin, vMax, hMin, hMax);
  // return;

  vert = map(vert, -2048, 2048, -255, 255);  // map to forward speed
  horz = map(horz, -2048, 2048, -100, 100);  // map to rotate speed, not turn speed

  if (horz == 0) {
    setMotorSpeed(vert, vert);
  } else if (horz < 0) {    // left
    if (vert == 0) {
      setMotorSpeed(horz, -horz);  // counter-clockwise: speed1 < 0 and speed2 > 0
    } else {
      setMotorSpeed(vert/2, vert);   // use different vertical speed on two wheel to make sure for turn
    }
  } else {  // horz > 0  // right
    if (vert == 0) {
      setMotorSpeed(horz, -horz);  // clockwise: speed1 > 0 and speed2 < 0
    } else {
      setMotorSpeed(vert, vert/2);
    }
  }
}

void loop() {
  if (NotLoop > 0) {
    return;
    // while(1);
  }

  ctrlSpeed();

}