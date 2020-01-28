#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster
import numpy as np

class OdomNode:

    def __init__(self):
        self.const = 0.05235987755
        self.left_value = None
        self.rigth_value = None
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.l_vel = 0.0
        self.a_vel = 0.0
        self.heading = 0
        self.left_sub = rospy.Subscriber("/left_encoder_value", Int64, self.callback_left_encoder, buff_size = 30)
        self.right_sub = rospy.Subscriber("/right_encoder_value", Int64, self.callback_right_encoder, buff_size = 30)
        self.odom_publisher = rospy.Publisher("odom", Odometry, queue_size=50)
        self.last_time = rospy.Time.now()
        self.odom_broadcaster = TransformBroadcaster()

    def callback_left_encoder(self, msg):
        self.left_value = msg.data
        pass

    def callback_right_encoder(self, msg):
        self.right_value = msg.data
        self.publish_odometry()
        pass

    def publish_odometry(self):
        current_time = rospy.Time.now()
        omega_l = (self.left_value * self.const)
        omega_r = (self.right_value * self.const)        
        self.l_vel = ((omega_l + omega_r)/2.0)*0.062
        self.a_vel = -(omega_l - omega_r)*(0.062/0.45)
        vel_dt = (current_time - self.last_time).to_sec()
        

        delta_heading = self.a_vel * vel_dt
        delta_x = (self.l_vel * np.cos(self.heading)) * vel_dt
        delta_y = (self.l_vel * np.sin(self.heading)) * vel_dt
        #print(self.x_pos, self.y_pos, self.heading, self.l_vel, self.a_vel, self.left_value, self.right_value, vel_dt)
        self.x_pos += delta_x
        self.y_pos += delta_y
        self.heading += delta_heading # TF2 heading to quaternion
        quat = quaternion_from_euler(0, 0, self.heading)

        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"
        odom_trans.transform.translation.x = self.x_pos
        odom_trans.transform.translation.y = self.y_pos
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation.x = quat[0]
        odom_trans.transform.rotation.y = quat[1]
        odom_trans.transform.rotation.z = quat[2]
        odom_trans.transform.rotation.w = quat[3]        
        #self.odom_broadcaster.sendTransform(odom_trans)
        self.odom_broadcaster.sendTransformMessage(odom_trans)
        
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x_pos
        odom.pose.pose.position.y = self.y_pos
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.pose.covariance[0] = 0.0001
        odom.pose.covariance[7] = 0.0001
        odom.pose.covariance[35] = 0.0001
        odom.twist.twist.linear.x = self.l_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.a_vel
        odom.twist.covariance[0] = 0.0001
        odom.twist.covariance[7] = 0.0001
        odom.twist.covariance[35] = 0.0001
        self.odom_publisher.publish(odom)

        self.last_time = current_time

if __name__ == "__main__":
    rospy.init_node("eds_odometry_node")
    odom_obj = OdomNode()
    rospy.spin()



