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
#include <TouchScreen.h>
#include <stdint.h>
#include <Wire.h>
#include <pidautotuner.h>
#include <Filters.h>
#include <Filters/Butterworth.hpp>

/// led pin output
///@{
#define LEDPORT     PORTB
#define LEDDDR      DDRB
#define SETLEDDDR   LEDDDR |= B00000001
#define SETLEDPORT  LEDPORT |= B00000001
#define CLEARLEDPORT  LEDPORT &= !(B00000001)
///@}

/// touch screen input 
///@{
int yLow  = A5;
int yHigh = A3;

int xLow  = A2;
int xHigh = A4;

TouchScreen ts = TouchScreen(xHigh, yHigh, xLow, yLow, 495);
TSPoint p;

uint16_t homeX = 536;            // raw data value for center of touchscreen
uint16_t homeY = 539;            // raw data value for center of touchscreen

unsigned int noTouchCount = 0; //viariable for noTouch

double  inputX[]    = {0,0,0}; 
double  inputXout[] = {0,0,0};
double  inputY[]    = {0,0,0};
double  inputYout[] = {0,0,0};
///@}

/// for FREERTOS scheduling
///@{
TaskHandle_t RT1handle, startuphandle, readRemotehandle, readTouchhandle, turnToAnglehandle, readPotshandle;
              
void RT1( void *pvParameters );
void startup( void *pvParameters );
void readRemote( void *pvParameters );
void readTouch( void *pvParameters );
void turnToAngle( void *pvParameters );
void readPots( void *pvParameters );
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

int servoXpin = 6;
int servoYpin = 5;

double servoXangle;
double servoYangle;

int flatX = 97;
int flatY = 86;

#define MIN_PULSE_WIDTH      650
#define MAX_PULSE_WIDTH      2350
#define FREQUENCY            50
///@} 

/// PID control
///@{
double setpointX;
double setpointY;
double Kp = 60;
double Kd = 10;
double Ki = 0.00;

int Ts = 25; 
unsigned long Stable=0; 

PID pidX(&inputXout[0], &servoXangle, &setpointX, Kp, Ki, Kd, DIRECT);
PID pidY(&inputYout[0], &servoYangle, &setpointY, Kp, Ki, Kd, DIRECT);

float b[] = {0.00552119, 0.01104239, 0.00552119};
float a[] = {1.77908235, -0.80116713};

///@} 
