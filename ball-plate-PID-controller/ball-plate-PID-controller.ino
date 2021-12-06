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
  xTaskCreate(turnToAngle, "motorTest", 200, NULL, 2, &turnToAnglehandle);
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
  // set servo to 90 degrees
  servoX.attach(servoXpin);
  servoY.attach(servoYpin);
  servoX.write(90);
  servoY.write(90);
  vTaskDelay(500 / portTICK_PERIOD_MS);
//  servoX.detach();
//  servoY.detach();

//  // set motor timers up 
//  SETMOTOR1DDR;
//  SETMOTOR2DDR;
//  TIMER4ON;
//  TIMER3ON;
//  TIMER4REGA = 0; TIMER4REGB = 0;
//  TIMER3REGA = 0; TIMER3REGB = 0;
//  TCCR4A = (TCCR4A & B00111100) | B10000010;   //Phase and frequency correct, Non-inverting mode, TOP defined by ICR1
//  TCCR4B = (TCCR4B & B11100000) | B00010001;   //No prescale
//  TCCR3A = (TCCR3A & B00111100) | B10000010;   //Phase and frequency correct, Non-inverting mode, TOP defined by ICR1
//  TCCR3B = (TCCR3B & B11100000) | B00010001;   //No prescale
//  ICR4 = 0xFFFF; 
//  ICR3 = 0xFFFF;
//  TIMERCOMPARE4A = (CLOCKFREQ / (TWO*666)) + 1;
//  TIMERCOMPARE3A = (CLOCKFREQ / (TWO*666)) + 1;     

    // setpoints
    setpointX = 440;
    setpointY = 530;
  while (1) {
    vTaskDelay(50 / portTICK_PERIOD_MS);
    servoX.write(servoXangle);
    servoY.write(servoYangle);
//    if (inputX == setpointX) {
//          TIMERCOMPARE4A = (CLOCKFREQ / (TWO*666)) + 1; // x 
//       } else if (inputX > setpointX) {
//          TIMERCOMPARE4A = (CLOCKFREQ / (TWO*600)) + 1;
//       } else if (inputX < setpointX) {
//          TIMERCOMPARE4A = (CLOCKFREQ / (TWO*700)) + 1;
//       }
//
//       if (inputY == setpointY) {
//          TIMERCOMPARE3A = (CLOCKFREQ / (TWO*666)) + 1; // x 
//       } else if (inputY > setpointY) {
//          TIMERCOMPARE3A = (CLOCKFREQ / (TWO*600)) + 1;
//       } else if (inputY < setpointY) {
//          TIMERCOMPARE3A = (CLOCKFREQ / (TWO*700)) + 1;
//       }
  }
}

/** 
 *  pid(void *p)
 *     This funciton computes the pid output.
 *     @author Connor Lowe
 */
void pid(void *pvParameters) {
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidX.SetTunings(Kp, Ki, Kd);   
  pidY.SetTunings(Kp, Ki, Kd);  
  vTaskDelay(500 / portTICK_PERIOD_MS); 
  while (1) {
    pidX.Compute();
    pidY.Compute();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

/** 
 *  readTouch(void *p)
 *     Read inputs from touch screen.
 *     @author Connor Lowe
 */
void readTouch(void *pvParameters) {  
  while (1) {
    // Read y
      // read from this pin
      pinMode(xHigh, INPUT);
      
      // make tristate
      pinMode(xLow, INPUT);
      digitalWrite(xLow, LOW);
  
      // form a voltage divider
      pinMode(yHigh, OUTPUT);
      pinMode(yLow, OUTPUT);
      digitalWrite(yHigh, HIGH);
      digitalWrite(yLow,  LOW);
  
      // read voltage for y in
      inputY = analogRead(xHigh);

    // Read x
      // read from this pin
      pinMode(yHigh, INPUT);
      
      // make tristate
      pinMode(yLow, INPUT);
      digitalWrite(yLow, LOW);
  
      // form a voltage divider
      pinMode(xHigh, OUTPUT);
      pinMode(xLow, OUTPUT);
      digitalWrite(xHigh, HIGH);
      digitalWrite(xLow,  LOW);
  
      // read voltage for y in
      inputX = analogRead(yHigh);
      vTaskDelay(50 / portTICK_PERIOD_MS);

    Serial.println(inputY);
    Serial.println(inputX);
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
          break;

          case 0xFF629D:           // press 2
          break;

          case 0xFFE21D:           // press 3
          break;

          case 0xFF22DD:           // press 4
          break;

          case 0xFF025D:           // press 5
          break;

          case 0xFFC23D:           // press 6
          break;

          case 0xFFE01F:           // press 7
          break;

          case 0xFFA857:           // press 8
          break;

          case 0xFF906F:           // press 9
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
