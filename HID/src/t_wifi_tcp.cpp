#include <Arduino.h>
// #include <string.h>
#include <ESP8266WiFi.h>

// const char* ssid = "R1216_2.4GHz"; // key in your own SSID
// const char* password = "r121612321"; // key in your own WiFi access point password
const char* ssid = "LCH-DELL"; // key in your own SSID
const char* password = "lch12345678"; // key in your own WiFi access point password

// const char* host = "192.168.1.4";
const char* host = "192.168.137.1";
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

  // Use WiFiClient class to create TCP connections
  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    ++NotLoop;
    return;
  }

  Serial.println("server connected");
  Serial.println();
}

void loop() {
  // delay(1000);  // Delay does not make the Arduino consume less energy.
  if (NotLoop > 0)
    return;

  // This will send the request to the server
  // client.println("shit...");
  client.write("shit...");
  Serial.println("Sent sth");

  int readLength = 0;
  while (readLength = client.available(), readLength == 0) {}  // wait for server's resp
  Serial.printf("read bytes available=%d\r\n", readLength);

  char readData[1024];
  // client.readBytesUntil('\n', readData, 8);  // // block with timeout. In some test tool, they don't send newline char...? So it needs to wait the \n for about several sec (may not be 1s described in doc)
  client.read(readData, readLength);
  readData[readLength] = 0;
  Serial.println(readData);

  // String line = client.readString();  // block with timeout
  // String line = client.readStringUntil('\n');  // block with timeout
  // Serial.println(line);

  // Read all the lines of the reply from server and print them to Serial
  // bool bGotResp = false;
  // while (client.available()) {
  //   if (!bGotResp) {
  //     Serial.println("Got server's Resp:");
  //     bGotResp = true;
  //   }

  Serial.println("closing connection");
  client.stop();
  ++NotLoop;
}