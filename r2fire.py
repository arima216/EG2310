# TO DO
# Set up the servo motor code ... copy from rpi
# Integrate it so that auto_nav stops when this code is running and resumes when code stops running

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool, String
import time
#import busio
#import board
#import #Thermo sensor
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
#setup servo and dc pins
servo_pin = 15
DC_pin1 = 23
DC_pin2 = 24

    def dc_start():
    	# setup dc motors
        GPIO.output(DC_pin1, GPIO.HIGH)
        GPIO.output(DC_pin2, GPIO.HIGH)
            

    def release():
        p = GPIO.PWM(servo_pin, 50)
# Set servo to 90 degrees as it's starting position 
        p.start(7.5)
        p.ChangeDutyCycle(2.5) #loading position 
        time.sleep(0.1) #delay .1 second 
        p.ChangeDutyCycle(9.5) #firing position 
        time.sleep(1) #delay 1 second again
        p.ChangeDutyCycle(7.5) #resting position 
        time.sleep(1)
        
    def shoot():
        #if correct_aim = True
        self.dc_start()
        for i in range (3)
            self.release()
        time.sleep(10)
        
try:
    while True:
        shoot()
except KeyboardInterrupt:
    p.stop()
    GPIO.output(DC_pin1, GPIO.LOW)
    GPIO.output(DC_pin2, GPIO.LOW)
    GPIO.cleanup()
