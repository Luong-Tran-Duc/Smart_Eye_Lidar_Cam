#!/usr/bin/env python3

import rospy	
from sensor_msgs.msg import Imu

class IMUFilter:
    def __init__(self):
        self.lp_cutoff = 5.0  # Cutoff frequency for the LPF (adjust as needed)
        self.dt = 0.005      # Time interval between measurements (adjust as needed)
        
        self.orientation_filtered = None
        self.angular_velocity_filtered = None
        self.linear_acceleration_filtered = None
        
        rospy.init_node('imu_filter', anonymous=True)
        rospy.Subscriber('/livox/imu', Imu, self.imu_callback)
        self.filtered_pub = rospy.Publisher('/filtered_imu', Imu, queue_size=10)

    def imu_callback(self, msg):
        if self.orientation_filtered is None:
            self.orientation_filtered = msg.orientation
            self.angular_velocity_filtered = msg.angular_velocity
            self.linear_acceleration_filtered = msg.linear_acceleration
        else:
            alpha = self.lp_cutoff * self.dt / (1 + self.lp_cutoff * self.dt)
            
            self.orientation_filtered.x = alpha * msg.orientation.x + (1 - alpha) * self.orientation_filtered.x
            self.orientation_filtered.y = alpha * msg.orientation.y + (1 - alpha) * self.orientation_filtered.y
            self.orientation_filtered.z = alpha * msg.orientation.z + (1 - alpha) * self.orientation_filtered.z
            self.orientation_filtered.w = alpha * msg.orientation.w + (1 - alpha) * self.orientation_filtered.w
            
            self.angular_velocity_filtered.x = alpha * msg.angular_velocity.x + (1 - alpha) * self.angular_velocity_filtered.x
            self.angular_velocity_filtered.y = alpha * msg.angular_velocity.y + (1 - alpha) * self.angular_velocity_filtered.y
            self.angular_velocity_filtered.z = alpha * msg.angular_velocity.z + (1 - alpha) * self.angular_velocity_filtered.z
            
            self.linear_acceleration_filtered.x = alpha * msg.linear_acceleration.x + (1 - alpha) * self.linear_acceleration_filtered.x
            self.linear_acceleration_filtered.y = alpha * msg.linear_acceleration.y + (1 - alpha) * self.linear_acceleration_filtered.y
            self.linear_acceleration_filtered.z = alpha * msg.linear_acceleration.z + (1 - alpha) * self.linear_acceleration_filtered.z
        
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.orientation = self.orientation_filtered
        filtered_msg.angular_velocity = self.angular_velocity_filtered
        filtered_msg.linear_acceleration = self.linear_acceleration_filtered
        
        self.filtered_pub.publish(filtered_msg)

if __name__ == '__main__':
    try:
        imu_filter = IMUFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
