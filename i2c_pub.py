import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Bool
import time 
import busio b
import board
import adafruit_amg8833


from pn532 import *

message_sent = 'Not Detected'

# Set up Thermal Camera
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c) 


class ThermalCamera(Node):

    def __init__(self):
        super().__init__('nfc_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def timer_callback(self):
        global message_sent 
        msg = String()
        msg.data = messgae_sent
        self.publisher_targetting.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        
    def find_taarget(self):
        global message_sent 
        target_found = False 
        while not target_found:
            deected = False
            for row in amg.pixels:
                print('[', end= " ")
                for temp in row:
                    if temp > detecting_threshold:
                        detected = True
                        target_found = True
                    print("{0:.1f}".format(temp), end=" ")
                print("]")
                print("\n")
            if detected == True:
                message_sent = 'Detected'
                self.timer_callback()
                print(" ")
                print("Detected!!")
                print("]")
                print("\n")
                time.sleep(1)       
        return True        
        

def main(args = None):
    rclpy.init(args = args)
    
    thermalcamera = ThermalCamera()
    thermalcamera.find_target()
    thermalcamera.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
