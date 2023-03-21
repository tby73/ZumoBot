import RPi.GPIO as GPIO
import time
import numpy as np

# DATA_OUT => GPIO 14
IR_SENSOR_DOUT = 14

# Check DO from IR-Sensor every 10 ms
CHECK_TIME = 0.01

def InitSystem():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IR_SENSOR_DOUT, GPIO.OUT)

def main():
    last_state = 0
    current_state = 1
    
    while True:
        # read sensor state
        current_state = GPIO.input(IR_SENSOR_DOUT)

        if current_state != last_state:

            # edge marking detected
            if current_state == 1:
                print("Edge detected, avoid")
            
            last_state = current_state
        
        # wait and check again
        time.sleep(CHECK_TIME)

if __name__ == "__main__":
    main()
    

