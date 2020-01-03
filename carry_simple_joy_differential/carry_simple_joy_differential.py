
#!/usr/bin/env python

__author__ = 'Turgay Senlet'

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

class Robot():

    def __init__(self):
        self.motor_pub = rospy.Publisher('motors', Vector3, queue_size=1)
        # Setup motor command
        self.sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
        self.cmd = Vector3()
        self.cmd.x = 0.0
        self.cmd.y = 0.0
        self.cmd.z = 0.0
        self.old_x = 0.0
        self.old_y = 0.0
        self.alpha = 0.03
        self.max = 1.2
        self.min = 0.5
        self.turn_ratio = 2.0
        self.motor_pub.publish(self.cmd)

    def joy_callback(self, msg):
        y = msg.axes[0]
        x = msg.axes[1]
        if x > self.alpha + self.old_x:
            self.old_x = self.old_x + self.alpha
        elif x < self.old_x - self.alpha:
            self.old_x = self.old_x - self.alpha
        else:
            self.old_x = x

        if y > self.alpha + self.old_y:
            self.old_y = self.old_y + self.alpha
        elif y < self.old_y - self.alpha:
            self.old_y = self.old_y - self.alpha
        else:
            self.old_y = y
        if (x < 0.05) & (x > -0.05):
            self.old_x = 0.01
        #else:
        #    self.old_x = self.old_x * (1.0-self.alpha) + x * self.alpha
        if (y < 0.05) & (y > -0.05):
            self.old_y = 0
        #else:
        #    self.old_y = self.old_y * (1.0-self.alpha) + y * self.alpha

        x = self.old_x
        y = self.old_y
        left = x - y/self.turn_ratio
        right = x + y/self.turn_ratio
        if left > self.max: left = self.max
        if left < -self.min: left = -self.min
        if right > self.max: right = self.max
        if right < -self.min: right = -self.min
        self.cmd.x = left
        self.cmd.y = right
        self.motor_pub.publish(self.cmd)

if __name__ == "__main__":
    rospy.init_node('robot')
    robot = Robot()
    rospy.spin()
