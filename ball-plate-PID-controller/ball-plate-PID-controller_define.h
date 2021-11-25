/**
 *   @file      ball-plate-PID-controller_define.h
 *   @author    Connor Lowe
 *   @date      11-November-2021
 *   @brief   Ball & Plate PID Controller
 *   
 *  This is the defines for  our ball and plate PID controller
 */

#include <Arduino_FreeRTOS.h>

/// led pin output
///@{
#define LEDPORT     PORTB
#define LEDDDR      DDRB
#define SETLEDDDR   LEDDDR |= B00100000
#define SETLEDPORT  LEDPORT |= B00100000
#define CLEARLEDPORT  LEDPORT &= !(B00100000)
///@}

/// for FREERTOS scheduling
///@{
TaskHandle_t RT1handle, startuphandle;
              
void RT1( void *pvParameters );
void startup( void *pvParameters );
///@}
