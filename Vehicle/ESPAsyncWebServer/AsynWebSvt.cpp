#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

const char* ssid = "YourSSID";
const char* password = "YourPassword";

AsyncWebServer server(80);
AsyncEventSource events("/events");  // SSE: Server-Sent Events
AsyncWebSocket ws("/ws");  // WebSocket for receiving commands

// Global variable for storing last received command
String lastCommand = "None";

// Handle WebSocket messages
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->opcode == WS_TEXT) {
        data[len] = '\0';
        lastCommand = String((char*)data);
        Serial.printf("WebSocket received: %s\n", data);

        // Example: If browser sends "LED_ON", respond
        if (lastCommand == "LED_ON") {
            ws.textAll("ESP32: LED is ON");
        }
    }
}

// WebSocket event handler
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("Client %u connected\n", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("Client %u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA) {
        // client->text("ACK");
        handleWebSocketMessage(arg, data, len);
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");

    if (!SPIFFS.begin()) {
        Serial.println("SPIFFS Mount Failed");
        return;
    }

    // Serve HTML, CSS, JS files
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/style.css", "text/css");
    });
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/script.js", "application/javascript");
    });

    // Handle HTTP GET request
    server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "ESP32: GET Request Received");
    });

    // Handle HTTP POST request
    server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (request->hasParam("message", true)) {
            String message = request->getParam("message", true)->value();
            Serial.printf("Received POST message: %s\n", message.c_str());
            request->send(200, "text/plain", "ESP32: POST Received - " + message);
        } else {
            request->send(400, "text/plain", "ESP32: Bad Request - Missing 'message' parameter");
        }
    });

    // SSE: Server-Sent Events
    server.addHandler(&events);

    // WebSockets
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);

    server.begin();
}

void loop() {
    ws.cleanupClients();

    // Send periodic updates via SSE
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 5000) {
        lastTime = millis();
        events.send("Temperature: 25°C", "temperature", millis());
    }
}
