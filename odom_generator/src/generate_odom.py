#!/usr/bin/env python
# license removed for brevity
# https://github.com/sungjik/my_personal_robotic_companion/blob/master/my_personal_robotic_companion/src/base_controller.cpp
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import tf
from math import atan2, sin, cos, degrees
from tf.transformations import euler_from_quaternion, quaternion_from_euler

encoder_pulse   = 121
wheel_diameter  = 0.17   # m
wheel_width     = 0.027   # m
track_width     = 0.25   # m
MAX_RPM         = 58
pi              = 3.1415926
two_pi          = 6.2831853
linear_scale_positive = 1.0
linear_scale_negative = 1.0
angular_scale_positive = 1.0
angular_scale_negative = 1.0
publish_tf = True

class OdomGenerator( object ):
    rpm_act1 = 0.0
    rpm_act2 = 0.0 
    rpm_dt = 0.0
    theta = 0.0
    x_pos = 0.0
    y_pos = 0.0
    rpm_time = rospy.Time()

    def __init__(self):
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        try:
            self.run()

        except rospy.ROSInterruptException:
            passs

    def callback(self, rpm):    
        self.rpm_act1 = rpm.vector.x
        self.rpm_act2 = rpm.vector.y
        self.rpm_dt = rpm.vector.z
        self.rpm_time = rpm.header.stamp        

    def run(self):
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10) # 10hz
        rospy.Subscriber("/rpm", Vector3Stamped, self.callback)    
        while not rospy.is_shutdown():
            # ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("rpm", n, d)
            current_time = rospy.get_rostime()
            dt = self.rpm_dt
            dxy_ave = (self.rpm_act1+self.rpm_act2)*dt*wheel_diameter*pi/(encoder_pulse*2)
            dth_odom = (self.rpm_act2-self.rpm_act1)*dt*wheel_diameter*pi/(encoder_pulse*track_width)
            dth = dth_odom
            if dth > 0: dth *= angular_scale_positive
            if dth < 0: dth *= angular_scale_negative
            if dxy_ave > 0: dxy_ave *= linear_scale_positive
            if dxy_ave > 0: dxy_ave *= linear_scale_negative

            dx = cos(dth) * dxy_ave
            dy = -sin(dth) * dxy_ave

            self.x_pos += (cos(self.theta) * dx - sin(self.theta) * dy)
            self.y_pos += (sin(self.theta) * dx + cos(self.theta) * dy)
            self.theta += dth

            if self.theta >= two_pi: self.theta -= two_pi
            if self.theta <= -two_pi: self.theta += two_pi
            # Yaw to quaternion
            odom_quat = Quaternion()
            quat = quaternion_from_euler(0.0, 0.0, self.theta)
            odom_quat.x = quat[0]
            odom_quat.y = quat[1]
            odom_quat.z = quat[2]
            odom_quat.w = quat[3]
            rospy.loginfo("x: " + `round(self.x_pos, 2)` + " y: " + `round(self.y_pos, 2)`+ " th: " + `round(self.theta, 2)`+ " o1: " + `round(self.rpm_act1, 2)`+ " o2: " + `round(self.rpm_act2, 2)` + " dt: " + `round(dt, 2)` )
            if publish_tf:
                t = TransformStamped()
                t.header.frame_id = "/odom"
                t.child_frame_id = "/camera_link"
                t.transform.translation.x = self.x_pos
                t.transform.translation.y = self.y_pos
                t.transform.translation.z = 0.0
                t.transform.rotation = odom_quat
                t.header.stamp = current_time
                br.sendTransform((t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
                    (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w), 
                    t.header.stamp,
                    t.child_frame_id, 
                    t.header.frame_id)

            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = "/odom"
            odom_msg.pose.pose.position.x = self.x_pos
            odom_msg.pose.pose.position.y = self.y_pos
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation = odom_quat

            if (self.rpm_act1 == 0) & (self.rpm_act2 == 0):
                odom_msg.pose.covariance[0] = 1e-9
                odom_msg.pose.covariance[7] = 1e-3
                odom_msg.pose.covariance[8] = 1e-9
                odom_msg.pose.covariance[14] = 1e6
                odom_msg.pose.covariance[21] = 1e6
                odom_msg.pose.covariance[28] = 1e6
                odom_msg.pose.covariance[35] = 1e-9
                odom_msg.twist.covariance[0] = 1e-9
                odom_msg.twist.covariance[7] = 1e-3
                odom_msg.twist.covariance[8] = 1e-9
                odom_msg.twist.covariance[14] = 1e6
                odom_msg.twist.covariance[21] = 1e6
                odom_msg.twist.covariance[28] = 1e6
                odom_msg.twist.covariance[35] = 1e-9
            else:
                odom_msg.pose.covariance[0] = 1e-3
                odom_msg.pose.covariance[7] = 1e-3
                odom_msg.pose.covariance[8] = 0.0
                odom_msg.pose.covariance[14] = 1e6
                odom_msg.pose.covariance[21] = 1e6
                odom_msg.pose.covariance[28] = 1e6
                odom_msg.pose.covariance[35] = 1e3
                odom_msg.twist.covariance[0] = 1e-3
                odom_msg.twist.covariance[7] = 1e-3
                odom_msg.twist.covariance[8] = 0.0
                odom_msg.twist.covariance[14] = 1e6
                odom_msg.twist.covariance[21] = 1e6
                odom_msg.twist.covariance[28] = 1e6
                odom_msg.twist.covariance[35] = 1e3

            if dt == 0: 
                vx = 0
            else:
                vx = dxy_ave/dt

            if dt == 0: 
                vth = 0
            else:
                vth = dth/dt
            odom_msg.child_frame_id = "/camera_link"
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.angular.z = dth
            # rospy.loginfo(odom_msg)
            self.odom_pub.publish(odom_msg);
            last_time = current_time;
            rate.sleep()

def main():
    rospy.init_node('odom_generator', anonymous=False)
    rospy.loginfo("Starting odom generator node...")

    try:
        odom_generator = OdomGenerator()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
    rospy.loginfo("Node ended.")

if __name__=='__main__':
    main()