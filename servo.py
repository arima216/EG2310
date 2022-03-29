import time
import RPi.GPIO as GPIO
# Set pin numbering convention 
GPIO.setmode(GPIO.BCM)
# Choose an appropriate pwm channel to be used to control the servo 
servo_pin = 15
# Set the pin as an output 
GPIO.setup(servo_pin, GPIO.OUT)
# Initialise the servo to be controlled by pwm with 50 Hz frequency 
p = GPIO.PWM(servo_pin, 50)
# Set servo to 90 degrees as it's starting position 
p.start(7.5)

def servo_fire():
    p.ChangeDutyCycle(2.5) #loading position 
    time.sleep(0.1) #delay .1 second 
    p.ChangeDutyCycle(9.5) #firing position 
    time.sleep(1) #delay 1 second again
    p.ChangeDutyCycle(7.5) #resting position 
    time.sleep(1)

try:
    while True:
        servo_fire()
except KeyboardInterrupt: 
    p.stop()
    GPIO.cleanup()