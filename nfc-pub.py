  GNU nano 4.8                                                                                    nfc-pub.py                                                                                               
"""
This example shows connecting to the PN532 with I2C (requires clock
stretching support), SPI, or UART. SPI is best, it uses the most pins but
is the most reliable and universally supported.
After initialization, try waving various 13.56MHz RFID cards over it!
"""

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pn532 import *

class nfcPublisher(Node):

    def __init__(self):
        super().__init__('nfc_publisher')
        self.publisher_ = self.create_publisher(String, 'nfc', 10)

    def nfc_pub(self):
        msg = String()
        msg.data = 'NFC Detected'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def startNFC():
    pn532 = PN532_I2C(debug=False, reset=20, req=16)
    ic, ver, rev, support = pn532.get_firmware_version()
    print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))
    pn532.SAM_configuration()

if __name__ == '__main__':
    try:
        startNFC()        

        nfc_publisher = nfcPublisher()

        print('Waiting for RFID/NFC card...')
        while True:
            # Check if a card is available to read``
            uid = pn532.read_passive_target(timeout=0.5)
            print('.', end="")
            # Try again if no card is available.
            if uid is None:
                continue
            print('Found card with UID:', [hex(i) for i in uid])
            nfc_publisher.nfc_pub
    except Exception as e:
        print(e)
    finally:
        GPIO.cleanup()

