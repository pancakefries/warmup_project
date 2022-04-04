#!/usr/bin/env python3 
import rospy
from geometry_msgs.msg import Twist, Vector3

class Square(object):

    def __init__(self):
        # Initialize ROS node /cmd_vel to send velocity commands
        rospy.init_node('cmd_vel')  
        # Initialize publisher to publish messages to /cmd_vel
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Setting the rate to 0.2 Hz so that the robot will switch from
        # moving forward to turning every 1/0.2 = 5 seconds
        self.rate = 0.2

        # Changing the value in self.lin_speed will change the size
        # of the square produced by the robot
        self.lin_speed = 0.2
        # An angular speed of 0.314 was calculated to turn the robot 
        # pi/2 radians (90 degrees) over the course of 5 seconds
        # (defined later as 'r')
        self.ang_speed = 0.314

    def run(self):
        # Robot has two possible actions, move forward or turn.
        # forward1 and angular1 will compose the turn message.
        forward1 = Vector3(x = 0., y = 0., z = 0.)
        angular1 = Vector3(x = 0., y = 0., z = self.ang_speed)

        # forward2 and angular2 will compose the forward message
        forward2 = Vector3(x = self.lin_speed, y = 0., z = 0.)
        angular2 = Vector3(x = 0., y = 0., z = 0.)

        # Combine the previously defined vectors
        turn_msg = Twist(linear = forward1, angular = angular1)
        forward_msg = Twist(linear = forward2, angular = angular2)
        
        r = rospy.Rate(self.rate)
        switch = 1

        # Turn in a square as long as rospy runs
        while not rospy.is_shutdown():
            # If switch = -1, move forward, if switch = 1, turn
            if switch < 0:
                self.publisher.publish(forward_msg)
            else:
                self.publisher.publish(turn_msg)
            # Multiplying switch by -1 will alternate its value between 
            # 1 and -1 after each iteration, allowing the message sent to 
            # alternate between moving forward and turning
            switch *= -1
            r.sleep()

if __name__ == '__main__':
    node = Square()
    node.run()