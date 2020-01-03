#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from math import atan2, sin, degrees
from tf.transformations import euler_from_quaternion, quaternion_from_euler

turn_around_speed = 0.4
turn_speed = 0.22
max_speed = 0.8
min_speed = 0.5
# Speed to back off when the marker is too close. Fixed speed, not adjusting by distance.
back_off_speed = 0.35
# The distance to ignore the marker after
max_distance = 6.0
# The distance where max speed is reached
mid_distance = 2.0
# The closes distance to care about, below stop
min_distance = 0.1
# Distance to back off when the marker is too close. Fixed speed, not adjusting by distance.
back_off_distance = 0.1
# Damping factor for angles, increase above 1 to reduce tracking oscilation
angle_smoothing = 1.4

class MarkerFollower( object ):
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    def getrotation (self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def __init__(self):
        #self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub = rospy.Publisher('/motors', Vector3, queue_size=10)
        try:
            self.follower()
        except rospy.ROSInterruptException:
            passs

    def callback(self, markers):    
        x = 0
        y = 0
        seq = 0
        current_speed = 0
        angle = 0
        speed_factor = 1
        upsidedown = 0
        for m in markers.markers:
            x = m.pose.pose.position.x
            y = m.pose.pose.position.y
            self.getrotation(m)
            # rospy.loginfo([self.yaw, self.pitch, self.roll, m.pose.pose.orientation.x])
            # if (self.yaw > 1.0) & (self.yaw < 2.2):
            #   rospy.loginfo('straight')
            if (self.yaw < -1.0) & (self.yaw > -2.2):
                rospy.loginfo('upside down')
                upsidedown = 1
            speed_factor = (x - min_distance) / (mid_distance - min_distance)
            if speed_factor > 1:
                 speed_factor = 1
            current_speed = max_speed * speed_factor
            seq = markers.header.seq            
        vel_msg = Vector3()

        angle = atan2(y, x)
        # Multiplier to turn speed when not moving. Motors may need more nudge to start when not moving.
        turn_speed_factor = 0.9
        speed_left = 0
        speed_right = 0
        mode = 0
        if current_speed < min_speed:
            #current_speed = 0
            turn_speed_factor = 3
            mode = 1
        if upsidedown:
                speed_left = -turn_around_speed
                speed_right = turn_around_speed
        else:
            if  (x != 0) & (x < max_distance) & (x > min_distance):
                speed_left = current_speed - sin(angle / angle_smoothing) * turn_speed * turn_speed_factor
                speed_right = current_speed + sin(angle / angle_smoothing) * turn_speed * turn_speed_factor
                mode = mode + 2
            elif (x != 0) & (x < back_off_distance):
                speed_left = -back_off_speed
                speed_right = -back_off_speed
                mode = mode + 4
        vel_msg.x = speed_left
        vel_msg.y = speed_right
        vel_msg.z = x
        self.pub.publish(vel_msg)
        # if upsidedown:
            # rospy.sleep(.3)
        # if x != 0:
            # rospy.loginfo(rospy.get_caller_id() + "Seq %d, Mark %f, SpeedLeft %f, SpeedRight %f, Angle %f, Mode %d, SpeedFactor %f",  seq, x, speed_left, speed_right, degrees(angle), mode, speed_factor)

    def follower(self):
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
        rospy.loginfo("Node started...")

        # rate = rospy.Rate(20) # 10hz    
        # while not rospy.is_shutdown():
        #     # hello_str = "hello world %s" % rospy.get_time()
        #     # rospy.loginfo(hello_str)
        #     # pub.publish(hello_str)
        #     rate.sleep()        

def main():
    rospy.init_node('marker_follower', anonymous=False)
    rospy.loginfo("Starting tracking node...")

    try:
        marker_follower = MarkerFollower()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
    rospy.loginfo("Node ended.")

if __name__=='__main__':
    main()