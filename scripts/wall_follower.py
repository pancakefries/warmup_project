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
        front = []
        back = []
        length = len(msg.ranges)
        q1 = length / 4
        q3 = q1 * 3
        for i in range(len(msg.ranges) - 1):
            if msg.ranges[i] != 0:
                if i < q1 or i > q3:
                    front.append([i, msg.ranges[i]])
                else: 
                    back.append([i, msg.ranges[i]])
                    
        min_front = 1000
        min_front_loc = 0
        for e in front:
            if e[1] < min_front:
                min_front = e[1]
                min_front_loc = e[0]
        min_back = 1000
        min_back_loc = 0
        for e in back:
            if e[1] < min_back:
                min_back = e[1]
                min_back_loc = e[0]

        if ((min_front - min_back) < -0.01) or (min_back < 0.4) :#or (min_front > 0.4):
            linspeed = self.linspeed #* (((min_front * 1) - 0.2) - (min_front % self.linspeed))
            angspeed = self.angspeed #* (min_right_loc / half)
        elif ((min_back - min_front) < -0.01) or (min_front < 0.4) :#or (min_back > 0.4):
            linspeed = self.linspeed #* (((min_back * 1) - 0.2) - (min_back % self.linspeed))
            angspeed = self.angspeed * -1#((half - (min_left_loc % half)) / half) * -1
        else: 
            linspeed = self.linspeed #* (((min_back * 1) - 0.2) - (min_back % self.linspeed))
            angspeed = 0

        # if ((min_front < min_back)) or (min_back < 0.2) or (min_front > 0.4):
        #     linspeed = self.linspeed #* (((min_front * 1) - 0.2) - (min_front % self.linspeed))
        #     angspeed = self.angspeed #* (min_right_loc / half)
        # elif ((min_back < min_front)) or (min_front < 0.2) or (min_back > 0.4):
        #     linspeed = self.linspeed #* (((min_back * 1) - 0.2) - (min_back % self.linspeed))
        #     angspeed = self.angspeed * -1#((half - (min_left_loc % half)) / half) * -1
        # else: 
        #     linspeed = self.linspeed #* (((min_back * 1) - 0.2) - (min_back % self.linspeed))
        #     angspeed = 0

        l = Vector3(x = linspeed, y = 0, z = 0)
        a = Vector3(x = 0, y = 0, z = angspeed)
        move_msg = Twist(linear = l, angular = a)
        self.publisher.publish(move_msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('wall_follower')
    node = Follower()
    node.run()