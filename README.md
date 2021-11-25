# ball-plate-PID-controller
Ball &amp; Plate PID Controller

This project is a work in progress. Two servo motors will balance a weighted ball on a 
  resistive touch screen. To minimize error a PID equation is implemented independently in 
  both the x and y directions. An IR remote also lets the user change the x and y setpoints. 

Components: Arduino Nano
            2 Servo motors
            4 pin resistive touch screen
            IR remote and reciever
            
This system utilized FREE RTOS for scheduling. 

One task will be to blink the onboard LED to ensure proper timing and an overall sanity check. (RT1)

The next step will be to implement the IR remote so that it can change the x and y setpoints.
