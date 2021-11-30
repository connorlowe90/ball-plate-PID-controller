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
double servoXangle;
double servoYangle;

#define MIN_PULSE_WIDTH      650
#define MAX_PULSE_WIDTH      2350
#define FREQUENCY            50

#define MOTOR1DDR  DDRH
#define MOTOR2DDR  DDRE
#define SETMOTOR1DDR MOTOR1DDR |= BIT_3
#define SETMOTOR2DDR MOTOR2DDR |= BIT_3
#define BIT_3 1<<3

#define TIMERPPR       PRR1

#define TIMER4_ON_BIT  1<<PRTIM4
#define TIMER3_ON_BIT  1<<PRTIM3
#define TIMER4ON      TIMERPPR &= !(TIMER4_ON_BIT) 
#define TIMER3ON      TIMERPPR &= !(TIMER3_ON_BIT)

#define TIMERCOMPARE4A   OCR4A
#define TIMERCOMPARE3A   OCR3A

#define TIMER4REGA     TCCR4A     
#define TIMER4REGB     TCCR4B
#define TIMER3REGA     TCCR3A     
#define TIMER3REGB     TCCR3B

#define WGM4_BIT_0      1<<WGM40
#define WGM4_BIT_1      1<<WGM41
#define WGM4_BIT_2      1<<WGM42
#define WGM4_BIT_3      1<<WGM43
#define WGM3_BIT_0      1<<WGM30
#define WGM3_BIT_1      1<<WGM31
#define WGM3_BIT_2      1<<WGM32
#define WGM3_BIT_3      1<<WGM33

#define PSCALE4_BIT_0   1<<CS40
#define PSCALE4_BIT_1   1<<CS41
#define PSCALE4_BIT_2   1<<CS42
#define PSCALE3_BIT_0   1<<CS30
#define PSCALE3_BIT_1   1<<CS31
#define PSCALE3_BIT_2   1<<CS32

#define COMPARE_MODE4   1<<COM4A0
#define COMPARE_MODE3   1<<COM3A0
#define SETTIMER4REGA    TIMER4REGA |= WGM4_BIT_0 | WGM4_BIT_1 | COMPARE_MODE4 
#define SETTIMER4REGB    TIMER4REGB |= WGM4_BIT_2 | WGM4_BIT_3 | PSCALE4_BIT_0
#define SETTIMER3REGA    TIMER3REGA |= WGM3_BIT_0 | WGM3_BIT_1 | COMPARE_MODE3
#define SETTIMER3REGB    TIMER3REGB |= WGM3_BIT_2 | WGM3_BIT_3 | PSCALE3_BIT_0

#define CLOCKFREQ      16000000
#define TWO            2
#define THREE          3
#define TEN            10
#define HUNDRED        100
#define THOUSAND       1000
#define COMPARE_2000   16000
#define COMPARE_293    27304
#define COMPARE_329    24316
#define COMPARE_261    30651
#define COMPARE_130    61538
#define COMPARE_196    40816
#define COMPARE_NONE   0
#define COMPARE_1      62500
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

//FilterOnePole xLowpass (LOWPASS, 45.0);
//FilterOnePole yLowpass (LOWPASS, 45.0); 
//
//FilterOnePole outxLowpass (LOWPASS, 25.0);
//FilterOnePole outyLowpass (LOWPASS, 25.0);

///@} 
