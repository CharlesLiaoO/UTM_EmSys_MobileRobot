// conclusion: cannot call loop() in a task, otherwise whole program crash
#include <Arduino.h>
void task1(void *pvParameters);

void setup() {
    Serial.begin(115200);
    xTaskCreatePinnedToCore(task1, "Task 1", 1000, NULL, 1, NULL, 1);
}

void loop() {
    // loop() 函数的代码
    Serial.println("loop() is running");
    delay(1000);
}

void task1(void *pvParameters) {
    Serial.println("task1 start");
    loop(); // 调用 loop() 函数
    Serial.println("task1 end");
}

