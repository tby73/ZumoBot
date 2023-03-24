import RPi.GPIO as GPIO
import time
import numpy as np

# GPIO Ultrasonic 1
GPIO_TRIGGER_1 = 5
GPIO_ECHO_1 = 6

# GPIO Ultrasonic 2
GPIO_TRIGGER_2 = 23
GPIO_ECHO_2 = 24

# GPIO Ultrasonic 3
GPIO_TRIGGER_3 = 23
GPIO_ECHO_3 = 24

# GPIO(14, 15) => IR-Sensors (Left/Right)
IR_SENSOR1_DOUT = 14
IR_SENSOR2_DOUT = 15

# IN_A and IN_B DC Motor 1
DC1_INPUT_A = 13 # Connect to IN1
DC1_INPUT_B = 19 # Connect to IN2

# IN_A and IN_B DC Motor 2
DC2_INPUT_A = 20 # Connect to IN3
DC2_INPUT_B = 21 # Connect to IN4

# sonic speed in cm for distance estimation and echo pulse delay
SONIC_SPEED_CM = 34300
PULSE_DELAY = 0.00001

# collision distance max
COLLISION_AVOIDANCE_THRESHOLD = 12

def InitSystem():
    GPIO.setmode(GPIO.BCM)

    # init Ultrasonic 1
    GPIO.setup(GPIO_TRIGGER_1, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_1, GPIO.IN)

    # init Ultrasonic 2
    GPIO.setup(GPIO_TRIGGER_2, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_2, GPIO.IN)

    # init Ultrasonic
    GPIO.setup(GPIO_TRIGGER_3, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_3, GPIO.IN)

    # init IR-Sensors
    GPIO.setup(IR_SENSOR1_DOUT, GPIO.IN)
    GPIO.setup(IR_SENSOR2_DOUT, GPIO.IN)

    # init DC1
    GPIO.setup(DC1_INPUT_A, GPIO.OUT)
    GPIO.setup(DC1_INPUT_B, GPIO.OUT)

    # init DC2
    GPIO.setup(DC2_INPUT_A, GPIO.OUT)
    GPIO.setup(DC2_INPUT_B, GPIO.OUT)

def SetMotorLogic(motor_index, speed):
    # set PWM DC1
    pwm_DC1_A = GPIO.PWM(DC1_INPUT_A, 50)
    pwm_DC1_B = GPIO.PWM(DC1_INPUT_B, 50)

    # set PWM DC2
    pwm_DC2_A = GPIO.PWM(DC2_INPUT_A, 50)
    pwm_DC2_B = GPIO.PWM(DC2_INPUT_B, 50)

    # setup logic for motor 1
    if motor_index == 1: 
        # positive speeds => forward mode (1 0)
        if speed > 0:
            GPIO.output(DC1_INPUT_A, True)
            GPIO.output(DC1_INPUT_B, False)
        # negative speeds => backward mode (0 0)
        else:
            GPIO.output(DC1_INPUT_A, False)
            GPIO.output(DC1_INPUT_B, True)
        
        # set speed to DC1
        pwm_DC1_A.start(np.abs(speed))
        pwm_DC1_B.start(np.abs(speed))

    # same logic for motor 2
    if motor_index == 2:
        if speed > 0:
            GPIO.output(DC2_INPUT_A, True)
            GPIO.output(DC2_INPUT_B, False)
        
        else:
            GPIO.output(DC2_INPUT_A, False)
            GPIO.output(DC2_INPUT_B, True)

        pwm_DC2_A.start(np.abs(speed))
        pwm_DC2_B.start(np.abs(speed))

def SetMotorMovement(speed_motor1, speed_motor2):
    SetMotorLogic(1, speed_motor1)
    SetMotorLogic(2, speed_motor2)

def GetDistanceCM(trigger, echo):
    # send pulse
    GPIO.output(trigger, True)

    # wait
    time.sleep(PULSE_DELAY)

    # shut trigger down 
    GPIO.output(trigger, False)

    start = time.time()
    end = time.time()

    # measure time difference
    while GPIO.input(echo) == 1:
        start = time.time()
    while GPIO.input(echo) == 0:
        end = time.time()

    # calculate distance from time difference and sonic speed
    time_difference = end - start
    distance = (time_difference * SONIC_SPEED_CM) / 2

    return distance

def main():
    safe_drive = True

    while True:
        if safe_drive:
            SetMotorMovement(120, 120)

        # read from Ultrasonic sensors
        distance_sensor1 = GetDistanceCM(GPIO_TRIGGER_1, GPIO_ECHO_1)
        distance_sensor2 = GetDistanceCM(GPIO_TRIGGER_2, GPIO_ECHO_2)
        distance_sensor3 = GetDistanceCM(GPIO_TRIGGER_3, GPIO_ECHO_3)

        # ultrasonic collision handler
        ultrasonic_collision = (distance_sensor1 <= COLLISION_AVOIDANCE_THRESHOLD or 
                                distance_sensor2 <= COLLISION_AVOIDANCE_THRESHOLD or 
                                distance_sensor3 <= COLLISION_AVOIDANCE_THRESHOLD)
        
        # if ring edge detected
        if GPIO.input(IR_SENSOR1_DOUT) or GPIO.input(IR_SENSOR2_DOUT):
            safe_drive = False

            SetMotorMovement(70, -70)
            time.sleep(1.5)

            safe_drive = True

        # check for collision via ultrasonics
        if ultrasonic_collision:
            safe_drive = False

            SetMotorMovement(-120, -120)
            time.sleep(1)
            SetMotorMovement(70, -70)

            safe_drive = True

