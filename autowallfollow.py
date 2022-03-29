# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time

# constants
rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
left_angle = -60
right_angle = 120
scanfile = 'lidar.txt'
mapfile = 'map.txt'
state_ = 0

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    print("euler")
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        print("init")
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])


    def odom_callback(self, msg):
        print("odom callback")
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self, msg):
        print("occ callback")
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        # np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        print("scan callback")
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

        self.bug_action()

    def change_state(self,state):
        global state_
        if state is not state_:
            print('State of Bot - %s' % (state))
            state_ = state
    
    def bug_action(self):
        global follow_dir

        b = 1  # maximum threshold distance
        a = 0.5  # minimum threshold distance
        twist = Twist()  # Odometry call for velocity
        linear_x = 0.0  # Odometry message for linear velocity will be called here.
        angular_z = 0.0  # Odometry message for angular velocity will be called here.

        if  np.nan_to_num(self.laser_range[left_angle], copy=False, nan=100) > b and np.nan_to_num(self.laser_range[front_angle], copy=False, nan=100)  > b:  # Loop 1
            self.change_state(0)
        elif np.nan_to_num(self.laser_range[left_angle], copy=False, nan=100)  > b and np.nan_to_num(self.laser_range[front_angle], copy=False, nan=100)  > a:
            self.change_state(4)
        elif np.nan_to_num(self.laser_range[left_angle], copy=False, nan=100)  and np.nan_to_num(self.laser_range[front_angle], copy=False, nan=100)  > a:
            self.change_state(2)
        elif np.nan_to_num(self.laser_range[left_angle], copy=False, nan=100)  and np.nan_to_num(self.laser_range[front_angle], copy=False, nan=100)  < a:
            self.change_state(3)
        else:
            print('Error')

    def find_wall(self):
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = 0.0
        return twist


    '''
    Function: turn_left:  This function publishes linear and angular velocities for turning left.
    '''


    def turn_left(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.3
        return twist


    '''
    Function: turn_right:  This function publishes linear and angular velocities for turning right.
    '''


    def turn_right(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.3
        return twist


    '''
    Function: move_ahead:  This function publishes linear and angular velocities for moving straight.
    '''


    def move_ahead(self):
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = 0.0
        return twist


    '''
    Function: move_diag_right:  This function publishes linear and angular velocities for moving diagonally right.
    '''


    def move_diag_right(self):
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = -0.3
        return twist


    '''
    Function: move_diag_left:  This function publishes linear and angular velocities for moving diagonally left.
    '''


    def move_diag_left(self):
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.3
        return twist

    def move(self):
        print("move")
        try:
        
            while rclpy.ok():

                twist = Twist()
                if state_ == 0:
                    twist = self.find_wall()
                elif state_ == 1:
                    twist = self.turn_right()
                elif state_ == 2:
                    twist = self.move_ahead()
                elif state_ == 3:
                    twist = self.turn_left()
                elif state_ == 4:
                    twist = self.move_diag_right()
                elif state_ == 5:
                    twist = self.move_diag_left()
                else:
                    print("Unknown!")
                    

                self.publisher_.publish(twist)
                rclpy.spin_once(self)

        except Exception as e:
            print(e)

        finally:
            self.stopbot()


    def stopbot(self):
        print("stopbot")
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)




def main(args=None):
    print("main")
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.move()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
