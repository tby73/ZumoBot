import cv2
import RPi.GPIO as GPIO
import numpy as np

SERVO_PIN = 11

def InitSystem():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)

def main():
    servo_pwm = GPIO.PWM(SERVO_PIN, 50)
    servo_pwm.start(0)


