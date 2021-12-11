# ball-plate-PID-controller
Ball &amp; Plate PID Controller

This project is a work in progress. Two servo motors will balance a weighted ball on a 
  resistive touch screen. To minimize error a PID equation is implemented independently in 
  both the x and y directions. An IR remote also lets the user change the x and y setpoints. 

Components: Arduino Mega
            2 Servo motors, with servo horns and arms, w/ decoupling capacitors
	    Center ball bearing for stability
            4 pin resistive touch screen
            IR remote and reciever
	    External Led and 220 ohm resistor for blink task
            
This system utilized FREE RTOS for scheduling. 

One task will be to blink the onboard LED to ensure proper timing and an overall sanity check. (RT1)

Another task reads from an IR remote so that it can change the x and y setpoints.

Another task reads inputs from the touch screen.

And another task sets the PID control equation.
