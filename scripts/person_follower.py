#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class Follower(object):

    def __init__(self):
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.linspeed = 0.1
        self.angspeed = 0.3

    def scan_callback(self, msg):
        right = []
        left = []
        length = len(msg.ranges)
        half = length / 2
        for i in range(len(msg.ranges) - 1):
            if msg.ranges[i] != 0:
                if i < half:
                    right.append([i, msg.ranges[i]])
                else: 
                    left.append([i, msg.ranges[i]])
                    
        min_right = 1000
        min_right_loc = 0
        for e in right:
            if e[1] < min_right:
                min_right = e[1]
                min_right_loc = e[0]
        min_left = 1000
        min_left_loc = 0
        for e in left:
            if e[1] < min_left:
                min_left = e[1]
                min_left_loc = e[0]

        if (min_right - min_left) < -0.05:
            linspeed = self.linspeed * (((min_right * 1) - 0.2) - (min_right % self.linspeed))
            angspeed = self.angspeed #* (min_right_loc / half)
        elif (min_left - min_right) < -0.05:
            linspeed = self.linspeed * (((min_left * 1) - 0.2) - (min_left % self.linspeed))
            angspeed = self.angspeed * -1#((half - (min_left_loc % half)) / half) * -1
        else: 
            linspeed = self.linspeed * (((min_left * 1) - 0.2) - (min_left % self.linspeed))
            angspeed = 0

        l = Vector3(x = linspeed, y = 0, z = 0)
        a = Vector3(x = 0, y = 0, z = angspeed)
        move_msg = Twist(linear = l, angular = a)
        self.publisher.publish(move_msg)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('person_follower')
    node = Follower()
    node.run()