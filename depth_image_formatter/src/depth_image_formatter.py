#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from math import atan2, sin, degrees
from sensor_msgs.msg import CompressedImage
# OpenCV
import cv2
import math
import numpy as np

class DepthImageFormatter( object ):
    def __init__(self):
        self.pub = rospy.Publisher('/aligned_depth/compressed', CompressedImage, queue_size=10)
        try:
            self.formatter()
        except rospy.ROSInterruptException:
            passs

    def callback(self, ros_data):
        rospy.loginfo("Image received...")
        np_arr = np.fromstring(ros_data.data, np.uint16)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.pub.publish(msg)
        rospy.loginfo("Image published.")

    def formatter(self):       
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.callback)        
        rospy.loginfo("Node started...")

        # rate = rospy.Rate(20) # 10hz    
        # while not rospy.is_shutdown():
        #     # hello_str = "hello world %s" % rospy.get_time()
        #     # rospy.loginfo(hello_str)
        #     # pub.publish(hello_str)
        #     rate.sleep()        

def main():
    rospy.init_node('depth_image_formatter', anonymous=False)
    rospy.loginfo("Starting depth image formatter node...")

    try:
        depth_image_formatter = DepthImageFormatter()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
    rospy.loginfo("Node ended.")

if __name__=='__main__':
    main()