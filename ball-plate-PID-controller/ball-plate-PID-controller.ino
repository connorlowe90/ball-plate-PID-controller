/**
 *   @file      ball-plate-PID-controller.ino
 *   @author    Connor Lowe
 *   @date      11-November-2021
 *   @brief   Ball & Plate PID Controller
 *   
 *  Components: Arudino Nano
 *              IR remote and sensor    - connected to D10/PB2 
 *              2 servo motors
 *              4 pin resistive touch screen
 */

/// defines and constants used for the program
#include "ball-plate-PID-controller_define.h"

/** 
 *  setup()
 *    Sets up the initial state of the program.
 *    Starts initial task
 *    @author Connor Lowe
 */
void setup() {
  xTaskCreate(startup, "Startup", 128, NULL, 2, &startuphandle);
  
  vTaskStartScheduler();
}

/** 
 *  loop()
 *    Does nothing - blank
 *    @author Connor Lowe
 */
void loop() {
}

/** 
 * startup(void *p)
 *    Starts PID controller.
 *     @author : Connor Lowe
 */
void startup(void *pvParameters) {
  xTaskCreate(RT1, "Blink", 100, NULL, 2, &RT1handle);
  while (1) {
  }
}

/** 
 *  RT1(void *p)
 *     This funciton blinks an on board led on for 500ms then off for 500ms.
 *     @author Connor Lowe
 */
void RT1(void *pvParameters) {
  SETLEDDDR;           
  while (1) {
    SETLEDPORT;
    vTaskDelay(500 / portTICK_PERIOD_MS);
    CLEARLEDPORT;
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}
