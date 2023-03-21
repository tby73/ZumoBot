import RPi.GPIO as GPIO
import time

# DATA_OUT => GPIO 14
IR_SENSOR_DOUT = 14

def InitSystem():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IR_SENSOR_DOUT, GPIO.OUT)

def main():
    InitSystem()
    
    while True:
        # read sensor state
        if GPIO.input(IR_SENSOR_DOUT):
            print("Edge detected, avoid")
        else:
            print("Inside marked space, continue")

if __name__ == "__main__":
    main()
    

