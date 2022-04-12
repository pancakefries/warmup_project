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
        # Set the maximum linear and angular speeds
        self.linspeed = 0.03
        self.angspeed = 0.3

    def scan_callback(self, msg):
        # By determining which side of the bot is closest to the wall, we can
        # tell it which direction to turn so it can follow it and maintain
        # distance
        front = []
        back = []
        # Since index 0 is at the front, the front facing half of the sensor is
        # from 3/4 to 1/4
        length = len(msg.ranges)
        q1 = length / 4
        q3 = q1 * 3
        for i in range(len(msg.ranges) - 1):
            # If there is an object close enough to be detected, add it to the
            # appropriate side
            if msg.ranges[i] != 0:
                if i < q1 or i > q3:
                    front.append([i, msg.ranges[i]])
                else: 
                    back.append([i, msg.ranges[i]])

        # We can compare each data point to find the minimum, i.e. which side is
        # closer to the wall
        min_front = 1000
        for e in front:
            if e[1] < min_front:
                min_front = e[1]
        min_back = 1000
        for e in back:
            if e[1] < min_back:
                min_back = e[1]

        # Determine if the front or back of the turtlebot is closer to the wall,
        # and then turn appropriately to maintain distance and follow
        # Making sure that the difference between min_front and min_back exceeds
        # some threshold prevents jitter
        if ((min_front - min_back) < -0.01) or (min_back < 0.3) :
            angspeed = self.angspeed
        elif ((min_back - min_front) < -0.01) or (min_front < 0.3) :
            angspeed = self.angspeed * -1
        else:
            angspeed = 0

        # Assemble the vector to be published to cmd_vel
        l = Vector3(x = self.linspeed, y = 0, z = 0)
        a = Vector3(x = 0, y = 0, z = angspeed)
        move_msg = Twist(linear = l, angular = a)
        self.publisher.publish(move_msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('wall_follower')
    node = Follower()
    node.run()