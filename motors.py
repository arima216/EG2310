import time
import RPi.GPIO as GPIO
# Set pin numbering convention 
GPIO.setmode(GPIO.BCM)

motorL_pin = 23
motorR_pin = 24

# Set the pin as an output 
GPIO.setup(motorL_pin, GPIO.OUT)
GPIO.setup(motorR_pin, GPIO.OUT)

def flywheel():
      GPIO.output(motorL_pin, GPIO.HIGH)
      GPIO.output(motorL_pin, GPIO.HIGH)

try:
    while True:
        flywheel()

except KeyboardInterrupt: 
    GPIO.output(motorL_pin, GPIO.LOW)
    GPIO.output(motorR_pin, GPIO.LOW)
    GPIO.cleanup()