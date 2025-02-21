// conclusion: no problem, not optimize the timerFlag out.

#include <Arduino.h>

bool timerFlag = false;  // No volatile

void IRAM_ATTR onTimer() {
    timerFlag = true;  // Modified in ISR
}

void setup() {
    Serial.begin(115200);

    auto timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000000, true);  // 1-second interval
    timerAlarmEnable(timer);
}

void loop() {
    if (timerFlag) {  // Compiler might optimize this out!
        Serial.println("Timer Interrupt Triggered!");
        timerFlag = false;
    }
    delay(1000);
}
