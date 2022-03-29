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

class FireBall(Node):
    def dc_start(self):
    	# setup dc motors
    	GPIO.setup(DC_pin1, GPIO.OUT)
    	GPIO.setup(DC_pin2, GPIO.OUT)
    	pwm1 = GPIO.PWM(DC_pin1, 100)
    	pwm1.start(0)
    	pwm2 = GPIO.PWM(DC_pin2, 100)
    	pwm2.start(0)
    	time.sleep(5)
    	
    	GPIO.output(DC_pin1, True)
    	GPIO.output(DC_pin1, True)
    	pwm1.ChangeDutyCycle(80)
    	pwm2.ChangeDutyCycle(80)
    	time.sleep(5)
    
    def release(self):
        # setup servo
        for i in range(3):
        
            GPIO.setup(servo_pin, GPIO.OUT)
            p = GPIO.PWM(servo_pin, 50)
            p.start(2.5)

            degree = 20

            servo_value = degree/90 * 5 + 2.5
            p.ChangeDutyCle(servo_value)
            time.sleep(1)
        
            degree = -20
        
            servo_value = degree/90 * 5 + 2.5
            p.ChangeDutyCle(servo_value)
            print(i)
            time.sleep(2)
        
    def shoot(self):
        #if correct_aim = True
        self.dc_start()
        self.release()
        
def main(args=None):
    rclpy.init(args=args)
    
    fire = FireBall()
    fire.shoot()
    
if __name__ == '__main__':
    main()
