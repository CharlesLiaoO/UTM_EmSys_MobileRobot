
#include <WiFi.h>

WiFiServer server(2020);

void setup() {
  Serial.begin(115200);
  Serial.println("Attempting to connect to WPA network...");

  WiFi.begin("R1216_2.4GHz", "r121612321");
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected as " + WiFi.localIP().toString());

  server.begin();
}

WiFiClient client;

void loop() {
//   if (server.hasClient()) {  // hasClient(*Incoming*). Works fine, but not Arduino standard API
//     Serial.println("hasClient");
//     if (!client) {
//       client = server.available();
//       Serial.println("New client connected: " + client.remoteIP().toString());
//     } else {
//         Serial.println("New client but existing client: " + client.remoteIP().toString());
//     }
//   } else {
//     Serial.println("Not hasClient(Incoming)");
//   }

  if (client) {  // == client.connected()
    while (client.available()) {
        auto str = client.readStringUntil('\n');
        Serial.println(str);
        client.write("ssssssssssss\n");
    }
  } else {
    client = server.available();  // try get new client
    if (client)
        Serial.println("New client connected: " + client.remoteIP().toString());
  }

  delay(1000);
}
