#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class Follower(object):

    def __init__(self):
        # Set up Subscriber and Publisher to read LiDAR data and send velocity
        # commands respectively
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # Set the maximum linear and angular speeds, as well as the distance
        # to the object from which the turtlebot should stop
        self.linspeed = 0.03
        self.angspeed = 0.3
        self.stop_dist = 0.2

    def scan_callback(self, msg):
        # By determining which side of the turtlebot the nearest object is on,
        # we can tell the bot to turn in that direction to follow it
        right = []
        left = []
        length = len(msg.ranges)
        # Splitting the data in half will give the side the object is on,
        # since 0 is the front-most point
        half = length / 2
        for i in range(len(msg.ranges) - 1):
            # If there's something close enough to be detected, add it
            # to the appropriate side
            if msg.ranges[i] != 0:
                if i < half:
                    right.append([i, msg.ranges[i]])
                else: 
                    left.append([i, msg.ranges[i]])
                    
        # We can compare each data point to find the minimum, i.e. the closest object
        min_right = 1000
        for e in right:
            if e[1] < min_right:
                min_right = e[1]
        min_left = 1000
        for e in left:
            if e[1] < min_left:
                min_left = e[1]

        # Determine which way to turn depending on which side has a closer object,
        # Making sure the difference between min_right and min_left exceeds some threshold
        # stops the turtlebot from jittering and allows it to travel straight if the
        # closest object is mostly straight ahead
        if (min_right - min_left) < -0.05:
            # Scaling the linear speed allows the bot to slowly come to a stop
            linspeed = self.linspeed * (((min_right) - self.stop_dist) - (min_right % self.linspeed))
            angspeed = self.angspeed
        elif (min_left - min_right) < -0.05:
            linspeed = self.linspeed * (((min_left) - self.stop_dist) - (min_left % self.linspeed))
            angspeed = self.angspeed * -1
        else: 
            linspeed = self.linspeed * (((min_left) - self.stop_dist) - (min_left % self.linspeed))
            angspeed = 0

        # Assemble the vector to publish to cmd_vel
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