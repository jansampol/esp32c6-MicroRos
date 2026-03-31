// #include <Arduino.h>
// #include "esp_timer.h"
// #include "RobotController/RobotController.h"
// #include "MicroRosController/MicroRosController.h"

// RobotController* robotController = nullptr;
// MicroRosController* microRosController = nullptr;
// esp_timer_handle_t motorTimer = NULL;

// void IRAM_ATTR onMotorTimer(void* arg) {
//     if (robotController != nullptr) {
//         robotController->service();
//     }
// }

// void setupMotorTimer() {
//     const esp_timer_create_args_t timer_args = {
//         .callback = &onMotorTimer,
//         .name = "motor_timer"
//     };
//     esp_timer_create(&timer_args, &motorTimer);
//     esp_timer_start_periodic(motorTimer, 1000);
// }

// //void stopMotorTimer(){
// //    esp_timer_stop(motorTimer);
// //    esp_timer_delete(motorTimer);
// //}

// void setup() {
//     Serial.begin(115200);
//     delay(3000);

//     Serial.println("===== SETUP START =====");

//     robotController = new RobotController();
//     robotController->begin();

//     setupMotorTimer();

//     microRosController = new MicroRosController();
//     microRosController->begin();

//     Serial.println("===== SETUP DONE =====");
// }

// void loop() {
//     if (microRosController != nullptr && robotController != nullptr) {
//         microRosController->update(*robotController);
//     }

//     if (robotController != nullptr) {
//         robotController->update();
//     }

//     RobotState state = robotController->getRobotState();
//     Serial.print("Joint steps: ");
//     for (auto s : state.jointSteps) {
//         Serial.print(s);
//         Serial.print(" ");
//     }
//     Serial.println();

//     delay(10);
// }

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    int counter = 0;

    while (1) {
        printf("ESP32-C6 ESP-IDF test OK | counter = %d\n", counter++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}