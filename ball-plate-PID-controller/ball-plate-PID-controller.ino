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
  xTaskCreate(readPots, "potentiometers", 200, NULL, 2, &readPotshandle);
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
  
  servoX.write(flatX);
  servoY.write(flatY);
  
  vTaskDelay(500 / portTICK_PERIOD_MS);
  
  servoX.detach();
  servoY.detach();  

  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);

  pidX.SetSampleTime(Ts); 
  pidY.SetSampleTime(Ts); 

  pidX.SetOutputLimits(flatX-5, flatX+5);
  pidY.SetOutputLimits(flatY-5, flatY+5);
  
  pidX.SetTunings(Kp, Ki, Kd);   
  pidY.SetTunings(Kp, Ki, Kd);  

    // setpoints
    setpointX = 450;
    setpointY = 530;
  while (1) {
    if (p.z > 20) {
      servoX.attach(servoXpin);
      servoY.attach(servoYpin);  
      pidX.Compute();
      pidY.Compute();
      noTouchCount = 0;
    } else {
      noTouchCount++; 
      if(noTouchCount == 150) {
          noTouchCount++; 
          servoX.attach(servoXpin);
          servoY.attach(servoYpin); 
          servoX.write(flatX);
          servoY.write(flatY);
          vTaskDelay(250 / portTICK_PERIOD_MS);
          servoX.detach();
          servoY.detach();
      }
    }
    servoX.write(servoXangle); 
    servoY.write(servoYangle);
    servoX.detach();
    servoY.detach(); 
  }
}

/** 
 *  pid(void *p)
 *     This funciton computes the pid output.
 *     @author Connor Lowe
 */
void readPots(void *pvParameters) { 
  
  vTaskDelay(500 / portTICK_PERIOD_MS); 
  
  while (1) {
//    Kp = map(analogRead(A15), 0, 1023, 0, 1000)/1000.0;
//    Kd = map(analogRead(A14), 0, 1023, 0, 1000)/1000.0;
//    Ki = map(analogRead(A13), 0, 1023, 0, 1000)/1000.0;
//    pidX.SetTunings(Kp, Ki, Kd);   
//    pidY.SetTunings(Kp, Ki, Kd); 
    Serial.print(setpointX);
    Serial.print("\t");
    Serial.println(inputXout[0]); 
  }
}

/** 
 *  readTouch(void *p)
 *     Read inputs from touch screen.
 *     @author Connor Lowe
 */
void readTouch(void *pvParameters) {  
  while (1) {
    p = ts.getPoint();   //measure pressure on plate
     
    // Read y
      // read from this pin
      pinMode(xHigh, INPUT_PULLUP);
      
      // make tristate
      pinMode(xLow, INPUT);
      digitalWrite(xLow, LOW);
  
      // form a voltage divider
      pinMode(yHigh, OUTPUT);
      pinMode(yLow, OUTPUT);
      digitalWrite(yHigh, HIGH);
      digitalWrite(yLow,  LOW);
  
      // read voltage for y in
      inputY[0] = analogRead(xHigh);
      inputYout[0] = a[0]*inputYout[1] + a[1]*inputYout[2] +
               b[0]*inputY[0] + b[1]*inputY[1] + b[2]*inputY[2];

    // Read x
      // read from this pin
      pinMode(yHigh, INPUT_PULLUP);
      
      // make tristate
      pinMode(yLow, INPUT);
      digitalWrite(yLow, LOW);
  
      // form a voltage divider
      pinMode(xHigh, OUTPUT);
      pinMode(xLow, OUTPUT);
      digitalWrite(xHigh, HIGH);
      digitalWrite(xLow,  LOW);
  
      // read voltage for y in
      inputX[0] = analogRead(yHigh);
      inputXout[0] = a[0]*inputXout[1] + a[1]*inputXout[2] +
               b[0]*inputX[0] + b[1]*inputX[1] + b[2]*inputX[2];
     

      for(int i = 1; i >= 0; i--){
        inputXout[i+1] = inputXout[i]; // store xi
        inputX[i+1] = inputX[i]; // store yi
        inputYout[i+1] = inputYout[i]; // store xi
        inputY[i+1] = inputY[i]; // store yi
      }

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
