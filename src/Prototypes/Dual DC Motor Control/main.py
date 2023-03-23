"""
This code can be used to control two DC-Motors using the MX1508 dual DC-Motor driver
using the Raspberry Pi 4 / NVIDIA Jetson. 

If mounted onto the ZumoBot (or any other two-wheel vehicle), the bot will make following movements from the code: 

1. Driving forward
2. Turn 90 Deg left
3. Driving forward
4. Turn 90 Deg left
5. Driving forward 
6. Turn 90 Deg left
7. Driving forward

                <-
POS1-----------POS4
|                |
|                |
|                |
|                |
|                |
|                |  ^
POS2-----------POS3 |
->
"""


import RPi.GPIO as GPIO
import time
import numpy as np

# IN_A and IN_B DC Motor 1
DC1_INPUT_A = 13 # Connect to IN1
DC1_INPUT_B = 19 # Connect to IN2

# IN_A and IN_B DC Motor 2
DC2_INPUT_A = 20 # Connect to IN3
DC2_INPUT_B = 21 # Connect to IN4

def InitSystem():
    # set Pi to BCM-Pinout
    GPIO.setmode(GPIO.BCM)

    # init DC1
    GPIO.setup(DC1_INPUT_A, GPIO.OUT)
    GPIO.setup(DC1_INPUT_B, GPIO.OUT)

    # init DC2
    GPIO.setup(DC2_INPUT_A, GPIO.OUT)
    GPIO.setup(DC2_INPUT_B, GPIO.OUT)

def SetMotorSpeed(motor, speed):
    # init PWM DC1
    pwm_DC1_A = GPIO.PWM(DC1_INPUT_A, 50)
    pwm_DC1_B = GPIO.PWM(DC1_INPUT_B, 50)

    # init PWM DC2
    pwm_DC2_A = GPIO.PWM(DC2_INPUT_A, 50)
    pwm_DC2_B = GPIO.PWM(DC2_INPUT_B, 50)

    if motor == 1:
        # move forward
        if speed > 0:
            GPIO.output(DC1_INPUT_A, True)
            GPIO.output(DC1_INPUT_B, False)

        # neg speed, move backward
        elif speed < 0:
            GPIO.output(DC1_INPUT_A, False)
            GPIO.output(DC1_INPUT_B, True)
        
        # set speed
        pwm_DC1_A.start(np.abs(speed))
        pwm_DC1_B.start(np.abs(speed))
    
    # same logic for motor 2
    if motor == 2:
        if speed > 0:
            GPIO.output(DC2_INPUT_A, True)
            GPIO.output(DC2_INPUT_B, False)

        elif speed < 0:
            GPIO.output(DC2_INPUT_A, False)
            GPIO.output(DC2_INPUT_B, True)

        pwm_DC2_A.start(np.abs(speed))
        pwm_DC2_B.start(np.abs(speed))

# Set Movement according to speed params
def SetMotorMovement(speed_1, speed_2):
    SetMotorSpeed(1, speed_1)
    SetMotorSpeed(2, speed_2)

# stop both motors
def Shutdown():
    GPIO.output(DC1_INPUT_A, False)
    GPIO.output(DC1_INPUT_B, False)

    GPIO.output(DC2_INPUT_A, False)
    GPIO.output(DC2_INPUT_B, False)

# break both motors
def Break():
    GPIO.output(DC1_INPUT_A, True)
    GPIO.output(DC1_INPUT_B, True)

    GPIO.output(DC2_INPUT_A, True)
    GPIO.output(DC2_INPUT_B, True)

def RoundMovement():
# move forward (POS1 -> POS2)
    SetMotorMovement(70, 70)

    # turn left (POS2_TURN_LEFT)
    SetMotorMovement(-70, 70)

    # move forward (POS2 -> POS3)
    SetMotorMovement(70, 70)

    # turn left (POS3_TURN_LEFT)
    SetMotorMovement(-70, 70)

    # move forward (POS3 -> POS4)
    SetMotorMovement(70, 70)

    # turn left (POS4_TURN_LEFT)
    SetMotorMovement(-70, 70)

    # move forward (POS4 -> POS1)
    SetMotorMovement(70, 70)

    # turn left (POS1_TURN_LEFT)    
    SetMotorMovement(-70, 70)    

    time.sleep(0.2)

def main():
    while True: 
        RoundMovement()

        # Shutdown motors if GPIO 12 button is pressed
        if GPIO.input(12):
            Shutdown()
            break
        
        # break if button GPIO 16 is pressed
        if GPIO.input(16):
            Break()

        if GPIO.input(3):
            RoundMovement()

if __name__ == "__main__":
    main()
    




