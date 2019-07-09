import rospy
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class OdomNode:

    def __init__():
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
        self.last_time = rospy.time.now()

    def callback_left_encoder(self, msg):
        self.left_value = msg.data
        pass

    def callback_right_encoder(self, msg):
        self.right_value = msg.data
        self.publish_odometry()
        pass

    def publish_odometry(self):
        current_time = rospy.Time.now()
        omega_l = (self.left_sub * self.const)
        omega_r = -(self.right_sub * self.const)        
        self.l_vel = ((omega_l + omega_r)/2.0)*0.06
        self.a_vel = - (omega_r - omega_l)*(0.06/0.245)
        vel_dt = (current_time - self.last_time).to_sec()

        delta_heading = self.a_vel * vel_dt
        delta_x = self.l_vel * cos(heading) - self.l_vel * sin(heading)
        delta_y = self.l_vel * sin(heading) + self.l_vel * cos(heading)

        self.x_pos += delta_x
        self.y_pos += delta_y
        heading += delta_heading # TF2 heading to quaternion

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x_pos
        odom.pose.pose.position.y = self.y_pos
        quat = Quaternion()


        self.last_time = current_time
        pass
