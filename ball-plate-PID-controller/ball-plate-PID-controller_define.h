/**
 *   @file      ball-plate-PID-controller_define.h
 *   @author    Connor Lowe
 *   @date      11-November-2021
 *   @brief   Ball & Plate PID Controller
 *   
 *  This is the defines for  our ball and plate PID controller
 */

#include <Arduino_FreeRTOS.h>
#include <IRremote.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Filters.h>

/// led pin output
///@{
#define LEDPORT     PORTB
#define LEDDDR      DDRB
#define SETLEDDDR   LEDDDR |= B00000001
#define SETLEDPORT  LEDPORT |= B00000001
#define CLEARLEDPORT  LEDPORT &= !(B00000001)
///@}

/// for FREERTOS scheduling
///@{
TaskHandle_t RT1handle, startuphandle, readRemotehandle, readTouchhandle, turnToAnglehandle, pidhandle;
              
void RT1( void *pvParameters );
void startup( void *pvParameters );
void readRemote( void *pvParameters );
void readTouch( void *pvParameters );
void turnToAngle( void *pvParameters );
void pid( void *pvParameters );
///@}

/// IR remote input
///@{
int RECV_PIN = 51;    
IRrecv irrecv(RECV_PIN);     
decode_results results;
///@} 

/// Servo motors
///@{
Servo servoX;
Servo servoY;
double servoXangle;
double servoYangle;
int SERVOXPIN = 44;
int SERVOYPIN = 46;
///@} 

/// touch screen input
///@{
double inputX;
double inputY;
///@} 

/// PID control
///@{
double setpointX;
double setpointY;
double Kp = 1;
double Ki = 0.15;
double Kd = 0.15;

PID pidX(&inputX, &servoXangle, &setpointX, Kp, Ki, Kd, DIRECT);
PID pidY(&inputY, &servoYangle, &setpointY, Kp, Ki, Kd, DIRECT);

FilterOnePole xLowpass (LOWPASS, 45.0);
FilterOnePole yLowpass (LOWPASS, 45.0); 

FilterOnePole outxLowpass (LOWPASS, 25.0);
FilterOnePole outyLowpass (LOWPASS, 25.0);

///@} 
