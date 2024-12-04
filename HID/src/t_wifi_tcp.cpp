#include <Arduino.h>
// #include <string.h>
#include <ESP8266WiFi.h>

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

int wifiClient_WaitReadAll(char * readData) {
  int readLength = 0;
  while (readLength = client.available(), readLength == 0) {}
  Serial.printf("read bytes available=%d\r\n", readLength);
  return client.read(readData, readLength);
}

void wifiClient_WaitReadCstr(char * cstr) {
  int size = wifiClient_WaitReadAll(cstr);
  cstr[size] = 0;
}

void loop() {
  if (NotLoop > 0)
    return;

  // This will send the request to the server
  client.write("shit...\n");
  Serial.println("Sent sth");

  client.setTimeout(5 * 1000);

  // char line[512];
  // read api: In some test tool, they don't send newline char...
  // wifiClient_WaitReadCstr(line);  // my test api...
  // client.available();  // available return is relevant to \0, \n or tcp timeout.
  // client.read(line, 8);  // no block
  // client.readBytes(line, 8);  // block with getting length bytes or timeout
  // client.readBytesUntil('\n', line, 8);  // block with timeout.
  // String line = client.readString();  // always return with timeout, without getting \0...
  String line = client.readStringUntil('\n');  // block with getting \n or timeout，without getting \0. If there's \0 before \n, it blocks with timeout...
  Serial.println(line);

  // Read all the lines of the reply from server and print them to Serial
  // String resp;
  // while (client.available()) {
  //   resp += client.readStringUntil('\n');
  // }
  // if (resp.isEmpty()) {
  //   Serial.println("Got server's Resp:");
  //   Serial.println(resp);
  // } else {
  //   Serial.println("Got server's Resp:");
  // }

  Serial.println("closing connection");
  client.stop();
  ++NotLoop;
}