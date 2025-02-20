#include <TaskScheduler.h>

Scheduler runner;
void task1Callback();
void task2Callback();
Task task1(500, TASK_FOREVER, &task1Callback, &runner);
Task task2(5000, TASK_FOREVER, &task2Callback, &runner);

void task1Callback() {
    static auto tb = 0;
    auto t = millis();
    Serial.println(t - tb);
    tb = t;
    Serial.println("Task 1 executed");
    task1.delay(1000);  // just adjust the delay for the next iteration, not blocking delay
    Serial.println("Task 1 delayed");
}
void task2Callback() { Serial.println("Task 2 executed"); }


void setup() {
    Serial.begin(115200);
    runner.enableAll();
}

void loop() {
    runner.execute();
}
