import RPi.GPIO as GPIO
import time
import datetime as dt
import matplotlib.pyplot as plt
import numpy as np

from matplotlib.animation import FuncAnimation

# GPIO Ultrasonic 1
GPIO_TRIGGER_1 = 5
GPIO_ECHO_1 = 6

# GPIO Ultrasonic 2
GPIO_TRIGGER_2 = 23
GPIO_ECHO_2 = 24

# Sonic Speed for Distance calculation and measure delay
SONIC_SPEED_CM = 34300
MEASURE_DELAY = 0.00001

COLLISION_THRESHOLD = 10

def InitUltrasonic():
    GPIO.setmode(GPIO.BCM)

    # init Sensor 1
    GPIO.setup(GPIO_TRIGGER_1, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_1, GPIO.IN)

    # init Sensor 2
    GPIO.setup(GPIO_TRIGGER_2, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_2, GPIO.IN)

def GetDistanceCM(trigger, echo):
    # send pulse
    GPIO.output(trigger, True)

    # wait for pulse back
    time.sleep(MEASURE_DELAY)

    # shut trigger down
    GPIO.output(trigger, False)

    start = time.time()
    end = time.time()

    # measure pulse back
    while GPIO.input(echo) == 0:
        start = time.time()
    while GPIO.input(echo) == 1:
        end = time.time()

    # get distance from time, use sonic speed for estimation
    time_difference = end - start
    distance = (time_difference * SONIC_SPEED_CM) / 2

    return distance


def PlotDistance(distance1, distance2):
    # setup plot figure
    fig = plt.figure()

    # create subplot
    ax = fig.add_subplot(1, 1, 1)

    # placeholder vectors
    xs = []
    distances_1 = []
    distances_2 = []

    def Animate(i, xs, distances_1, distances_2):
        # get distances
        distance1, distance2 = int(distance1), int(distance2)

        # Add x and measured distances to lists
        xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
        distances_1.append(distance1)
        distances_2.append(distance2)

        # Limit x and y (y => (4 cm to 4 m))
        xs = xs[-20:]
        ax.set_ylim(4, 400)

        # Draw x and y lists
        ax.clear()
        ax.plot(xs, distances_1)
        ax.plot(xs, distances_2)

        # Format plot
        plt.xticks(rotation=45, ha='right')
        plt.subplots_adjust(bottom=0.30)

        plt.title('Ultrasonic Measurements over Time')
        plt.ylabel('Distance [cm]')

    # init live animation
    animation = FuncAnimation(fig, Animate, fargs=(xs, distances_1, distances_2), interval=1000)
    
    # display
    plt.show()    

def main():
    InitUltrasonic()

    while True:
        # get distances from sensor
        distance_cm_sensor1 = GetDistanceCM(GPIO_TRIGGER_1, GPIO_ECHO_1)
        print(f"Gemossene Distanz(Sensor 1) [cm]: {distance_cm_sensor1}")

        distance_cm_sensor2 = GetDistanceCM(GPIO_TRIGGER_2, GPIO_ECHO_2)
        print(f"Gemossene Distanz(Sensor 2) [cm]: {distance_cm_sensor2}")

        # plot both measurements
        PlotDistance(distance_cm_sensor1, distance_cm_sensor2)

        # TODO: fusion with IR-Sensor to check if the vehicle isnt too 
        if distance_cm_sensor1 <= COLLISION_THRESHOLD or distance_cm_sensor2 <= COLLISION_THRESHOLD:
            print("AVOID, driving forward")


if __name__ == "__main__":
    main()

