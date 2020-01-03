#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from math import atan2, sin, degrees
import math
import numpy

class LaserFormatter( object ):
    def __init__(self):
        self.pub = rospy.Publisher('/scan_array', LaserScan, queue_size=10)
        try:
            self.formatter()
        except rospy.ROSInterruptException:
            passs

    def callback(self, scan):    
        # rospy.loginfo("Scan received.")
        data = numpy.array(scan.ranges)
        # rospy.loginfo(numpy.array(scan.ranges))
        # my_array_for_publishing = Float32MultiArray(data=data)
        for i in range(len(data)):
            if math.isnan(data[i]):
                if i>0:
                    data[i] = data[i-1]
                else:
                    data[i] = 0
        scan.ranges = data
        
        # my_array_for_publishing = Float32MultiArray(data=[3250,2682,6832,2296,8865,7796,6955,8236])
        self.pub.publish(scan)

    def formatter(self):       
        rospy.Subscriber("/scan", LaserScan, self.callback)        
        rospy.loginfo("Node started...")

        # rate = rospy.Rate(20) # 10hz    
        # while not rospy.is_shutdown():
        #     # hello_str = "hello world %s" % rospy.get_time()
        #     # rospy.loginfo(hello_str)
        #     # pub.publish(hello_str)
        #     rate.sleep()        

def main():
    rospy.init_node('laser_formatter', anonymous=False)
    rospy.loginfo("Starting laser formatter node...")

    try:
        laser_formatter = LaserFormatter()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
    rospy.loginfo("Node ended.")

if __name__=='__main__':
    main()