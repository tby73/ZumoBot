import RPi.GPIO as GPIO
import time 

# ultrasonic 1
GPIO_TRIGGER_1 = 5
GPIO_ECHO_1 = 6

# ultrasonic 2
GPIO_TRIGGER_2 = 23
GPIO_ECHO_2 = 24

# data out IR1
IR_SENSOR1_DOUT = 14

# data out IR2
IR_SENSOR2_DOUT = 15 

# Sonic speed for distance estimation + echo pulse delay
SONIC_SPEED_CM = 34300
PULSE_DELAY = 0.00001

# Collision avoidance 
COLLISION_AVOIDANCE_THRESHOLD = 10

def InitSystem():
    # set Pi to BCM-Pinout 
    GPIO.setmode(GPIO.BCM)

    # init Ultrasonic 1
    GPIO.setup(GPIO_TRIGGER_1, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_1, GPIO.IN)

    # init Ultrasonic 2
    GPIO.setup(GPIO_TRIGGER_2, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_2, GPIO.IN)

    # init IR-Sensors
    GPIO.setup(IR_SENSOR1_DOUT, GPIO.IN)
    GPIO.setup(IR_SENSOR2_DOUT, GPIO.IN)

def GetDistanceCM(trigger, echo):
    # send pulse
    GPIO.output(trigger, True)

    # wait 
    time.sleep(PULSE_DELAY)

    # shut trigger pulse down
    GPIO.output(trigger, False)

    # start time measurement
    start = time.time()
    end = time.time()

    # measure
    while GPIO.input(echo) == 0:
        start = time.time()
    while GPIO.input(echo) == 1:
        end = time.time()

    # calculate distance based on time difference between echo pulse and the sonic speed in cm 
    time_difference = end - start
    distance = (time_difference * SONIC_SPEED_CM) / 2

    return distance

def main():
    while True: 
        distance_sensor1 = GetDistanceCM(GPIO_TRIGGER_1, GPIO_ECHO_1)
        distance_sensor2 = GetDistanceCM(GPIO_TRIGGER_2, GPIO_ECHO_2)

        if distance_sensor1 <= COLLISION_AVOIDANCE_THRESHOLD or distance_sensor2 <= COLLISION_AVOIDANCE_THRESHOLD:
            if GPIO.input(IR_SENSOR1_DOUT) or GPIO.input(IR_SENSOR2_DOUT):
                # move backward
            # move forward
        

if __name__ == "__main__":
    main()

