#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)
    i = 1
    l = len(data.ranges)
    s = ""
    for x in data.ranges:
        x = x-0.4
        if i % 32 == 15:
            if math.isnan(x):
                x = 9
            if x>=9:
                x = 9
            
            xx = int(x*10.0)
            xs = str(xx)
            if xx < 10:
                xs = "0" + xs
            s = xs + s 
            print([str(i),str(x),xs])
        i = i + 1
    print s
    pub.publish(s)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print "Starting node"
    pub = rospy.Publisher('scan_string', String, queue_size=10)
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
