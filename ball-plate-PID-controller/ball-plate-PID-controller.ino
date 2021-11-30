/**
 *   @file      ball-plate-PID-controller.ino
 *   @author    Connor Lowe
 *   @date      11-November-2021
 *   @brief   Ball & Plate PID Controller
 *   
 *  Components: Arudino Mega
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
  Serial.begin(19200);
  while (!Serial) { 
  } 
  
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
  xTaskCreate(readRemote, "IR remote input", 200, NULL, 2, &readRemotehandle);
  xTaskCreate(readTouch, "touch screen input", 200, NULL, 2, &readTouchhandle);
  xTaskCreate(turnToAngle, "servo movement",200, NULL, 2, &turnToAnglehandle);
  xTaskCreate(pid, "pid computation", 200, NULL, 2, &pidhandle);
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


/** 
 *  turnToAngle(void *p)
 *     This funciton turns two servos to a given angle.
 *     @author Connor Lowe
 */
void turnToAngle(void *pvParameters) {
  SETMOTOR1DDR;
  SETMOTOR2DDR;
  TIMER4ON;
  TIMER3ON;
  TIMER4REGA = 0; TIMER4REGB = 0;
  TIMER3REGA = 0; TIMER3REGB = 0;
  TCCR4A = (TCCR4A & B00111100) | B10000010;   //Phase and frequency correct, Non-inverting mode, TOP defined by ICR1
  TCCR4B = (TCCR4B & B11100000) | B00010001;   //No prescale
  TCCR3A = (TCCR3A & B00111100) | B10000010;   //Phase and frequency correct, Non-inverting mode, TOP defined by ICR1
  TCCR3B = (TCCR3B & B11100000) | B00010001;   //No prescale
  ICR4 = 0xFFFF; 
  ICR3 = 0xFFFF;       
  while (1) {
    TIMERCOMPARE4A = (CLOCKFREQ / (TWO*666)) + 1;
    vTaskDelay(500 / portTICK_PERIOD_MS);
    TIMERCOMPARE4A = (CLOCKFREQ / (TWO*700)) + 1;
    vTaskDelay(500 / portTICK_PERIOD_MS);
    TIMERCOMPARE4A = (CLOCKFREQ / (TWO*600)) + 1;
    vTaskDelay(500 / portTICK_PERIOD_MS);

    TIMERCOMPARE3A = COMPARE_196;   
  }
}

/** 
 *  pid(void *p)
 *     This funciton computes the pid output.
 *     @author Connor Lowe
 */
void pid(void *pvParameters) {
  setpointX = 1023/2;
  setpointY = 1023/2;
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidX.SetTunings(Kp, Ki, Kd);   
  pidY.SetTunings(Kp, Ki, Kd);   
  while (1) {
    pidX.Compute();
    pidY.Compute();
  }
}

/** 
 *  readTouch(void *p)
 *     Read inputs from touch screen.
 *     @author Connor Lowe
 */
void readTouch(void *pvParameters) {  
  inputX = 1023/2;
  inputY = 1023/2;
  while (1) {
    inputX = analogRead(A0);
    inputY = analogRead(A1);
  }
}

/** 
 *  readRemote(void *p)
 *     This funciton reads the input from the IR remote and manipulates the setpoint.
 *     @author Connor Lowe
 */
void readRemote(void *pvParameters) {    
  irrecv.enableIRIn();  
  while (1) {
    if (irrecv.decode(&results)) {         // Returns 0 if no data ready, 1 if data ready.          
       Serial.println(results.value, HEX); //prints the value a a button press   

       // manipute the setpoints 
       switch (results.value) {
          case 0xFFA25D:           // press 1
            setpointX = 255/4;
            setpointY = 255/4;
          break;

          case 0xFF629D:           // press 2
            setpointX = 255/2;
            setpointY = 255/4;
          break;

          case 0xFFE21D:           // press 3
            setpointX = 3*255/4;
            setpointY = 255/4;
          break;

          case 0xFF22DD:           // press 4
            setpointX = 255/4;
            setpointY = 255/2;
          break;

          case 0xFF025D:           // press 5
            setpointX = 255/2;
            setpointY = 255/2;
          break;

          case 0xFFC23D:           // press 6
            setpointX = 3*255/4;
            setpointY = 255/2;
          break;

          case 0xFFE01F:           // press 7
            setpointX = 255/4;
            setpointY = 3*255/4;
          break;

          case 0xFFA857:           // press 8
            setpointX = 255/2;
            setpointY = 3*255/4;
          break;

          case 0xFF906F:           // press 9
            setpointX = 3*255/4;
            setpointY = 3*255/4;
          break;

          case 0xFF9867:           // press 0

          break;

          case 0xFF38C7:           // press ok

          break;

          case 0xFF10EF:           // press left

          break;

          case 0xFF18E7:           // press up

          break;

          case 0xFF5AA5:           // press right

          break;

          case 0xFF4AB5:           // press down

          break;
       }
       irrecv.resume(); // Restart the ISR state machine and Receive the next value     
    }    
  }
}
