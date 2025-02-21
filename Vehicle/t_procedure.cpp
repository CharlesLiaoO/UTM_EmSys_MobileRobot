#include "Arduino.h"
#include <Ticker.h>

void task1(void *pvParameters);

// ISR conclusion:
// ISR runs in the core of the task setup this ISR
// ISR can run after the task ended
// Serial.printf() in ISR sometimes cause crash due to timeout in ISR

const int pin2ISR = 4;
volatile int counter = 0;
void IRAM_ATTR handleInterrupt() {
    counter++;
    Serial.printf("Interrupt on Core: %d, Counter: %d\n", xPortGetCoreID(), counter);
}

void IRAM_ATTR onTimer() {
    Serial.printf("onTimer Interrupt on Core: %d\n", xPortGetCoreID());
}

// Ticker's callback always runs in core 0, regardless of the core setup it.
// Ticker uses ESP32’s FreeRTOS software timers, which are managed by the Timer Task.
// The Timer Task in ESP32 Arduino always runs on Core 0, because it is created by the ESP-IDF system scheduler.
Ticker ticker;
void tickerCallback() {
    Serial.printf("tickerCallback on Core: %d\n", xPortGetCoreID());
}

void setup() {
    Serial.begin(115200);
    Serial.printf("Setup running on Core: %d\n", xPortGetCoreID());

    pinMode(pin2ISR, INPUT_PULLUP);

    // setup ISR in core x:
    xTaskCreatePinnedToCore(task1, "Task 1", 1000, NULL, 1, NULL, 0);
}

void loop() {
    // Serial.println(getArduinoLoopTaskStackSize());  // 8192
    // Serial.printf("Free heap memory: %d bytes\n", esp_get_free_heap_size());  // 278352 bytes
    // Serial.printf("Largest free block: %d bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));  // 114676 bytes

    // Serial.printf("Loop running on Core: %d\n", xPortGetCoreID());
    delay(1000);
}

void task1(void *pvParameters) {
    Serial.println("task1 start");

    // hw_timer_t *timer = timerBegin(0, 80, true);  // hardware timer, 80 MHz / 80 = 1 MHz (1us per tick, 1us resolution)
    // timerAlarmWrite(timer, 1000000, true);  // 1 second (1,000,000 us) to Alarm
    // timerAttachInterrupt(timer, &onTimer, true);
    // timerAlarmEnable(timer);  // Enable timer

    ticker.attach_ms(1000, tickerCallback);

    attachInterrupt(digitalPinToInterrupt(pin2ISR), handleInterrupt, FALLING);

    // In core 1, can loop here without delay, not affect tasks(loop) in same core;
    // but in core 0, loop needs delay, otherwise crash
    // while(1) {
    //     delay(1000);
    // }

    Serial.println("task1 end");

    vTaskDelete(NULL);  // if end task without deleting task(itself), cause crash
}
