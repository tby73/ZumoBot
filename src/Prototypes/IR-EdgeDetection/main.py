import RPi.GPIO as GPIO
import time

# DATA_OUT => GPIO 14
IR_SENSOR_DOUT = 14

# Check DO from IR-Sensor every 10 ms
CHECK_TIME = 0.01

def InitSystem():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IR_SENSOR_DOUT, GPIO.OUT)

def main():
    InitSystem()
    
    while True:
        # read sensor state
        if GPIO.input(IR_SENSOR_DOUT):
            print(f"Edge detected, avoid")
        else:
            continue

if __name__ == "__main__":
    main()
    

