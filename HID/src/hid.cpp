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
}

// mcu input pins for joystick
const int joystick_Vert_Pin = 36;
const int joystick_Horz_Pin = 39;

void stopLoop(char * msg) {
  NotLoop++;
  Serial.println(msg);
  Serial.println("closing connection");
  client.stop();
}

void setMotorSpeed(float speed1, float speed2)
{
  client.printf("ms,%.3f,%.3f\n", speed1, speed2);
  // client.flush();

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
void ctrlSpeed() {
  // int vert = (analogRead(Joystick_Vert_Pin));
  // int horz = (analogRead(Joystick_Horz_Pin));
  int vert = map(analogRead(joystick_Vert_Pin), 0, 4095, -255, 255);  // map to forward speed
  int horz = map(analogRead(joystick_Horz_Pin), 0, 4095, -125, 125);  // map to rotate speed, not turn speed

  if (vert_b == vert && horz_b == horz)
    return;
  vert_b = vert;
  horz_b = horz;

  if (horz == 0) {
    setMotorSpeed(vert, vert);
  } else if (horz > 0) {    // left
    if (vert == 0) {
      setMotorSpeed(-horz, horz);  // use rotate speed
    } else {
      setMotorSpeed(vert/2, vert);   // use different vertical speed on two wheel to make sure for turn
    }
  } else {    // horz < 0    // right
    if (vert == 0) {
      setMotorSpeed(-horz, horz);
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