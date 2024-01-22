#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float64
import message_filters
import numpy as np


class Robot():
    def __init__(self):
        front_lidar = message_filters.Subscriber('diff_drive_robot/laser/scan', LaserScan)
        self.left_wheel_publisher = rospy.Publisher("robot_control/left_wheel_joint/command", Float64, queue_size=10)
        self.right_wheel_publisher = rospy.Publisher("robot_control/right_wheel_joint/command", Float64, queue_size=10)
        ts = message_filters.ApproximateTimeSynchronizer([front_lidar], 10, 0.1)
        ts.registerCallback(self.lidar_callback)
        self.control_speed = 0
        self.last_angle = 0

    def calculate_wheel_speeds(self, target_angle, max_wheel_speed):
        if target_angle == 0:
            target_angle = self.last_angle
        current_angle = target_angle
        target_angle_rad = np.deg2rad(target_angle)
        if not 86 <= target_angle <= 95:
            if self.control_speed - 0.1 > 10:
                self.control_speed -= 0.05
        else:
            if max_wheel_speed > self.control_speed:
                self.control_speed = min(max_wheel_speed, self.control_speed + 0.05)
            elif max_wheel_speed < self.control_speed:
                self.control_speed = max(max_wheel_speed, self.control_speed - 0.05)
            else:
                self.control_speed = max_wheel_speed

        if target_angle_rad < np.pi / 2:
            if 20 <= target_angle <= 50:
                left_wheel_speed = self.control_speed * (1 - (target_angle_rad - np.pi / 2) / (np.pi / 2))
                right_wheel_speed = self.control_speed
            else:
                left_wheel_speed = self.control_speed * (1 - (target_angle_rad - np.pi / 2) / (np.pi / 2))
                right_wheel_speed = self.control_speed
        else:
            if 130 <= target_angle <= 160:
                left_wheel_speed = self.control_speed
                right_wheel_speed = self.control_speed * (1 - (np.pi - target_angle_rad - np.pi / 2) / (np.pi / 2))
            else:
                left_wheel_speed = self.control_speed
                right_wheel_speed = self.control_speed * (1 - (np.pi - target_angle_rad - np.pi / 2) / (np.pi / 2))
        rospy.loginfo("%s", self.control_speed)
        if target_angle != 0:
            self.last_angle = target_angle
        if current_angle == 0:
            left_wheel_speed = -left_wheel_speed
            right_wheel_speed = - right_wheel_speed
        return left_wheel_speed, right_wheel_speed

    def find_largest_inf_gap(self, angles, values):
        max_gap_size = 0
        max_gap_middle = 0
        i = 0
        while i < len(values):
            if np.isinf(values[i]):
                start_index = i

                while i < len(values) and np.isinf(values[i]):
                    i += 1
                end_index = i - 1
                gap_size = end_index - start_index + 1
                if gap_size > max_gap_size:
                    max_gap_size = gap_size
                    max_gap_middle = (angles[start_index] + angles[end_index]) / 2
            else:
                i += 1

        return max_gap_middle

    def lidar_callback(self, data):
        ranges = data.ranges
        angles = 20 + np.arange(len(ranges)) * 2.8
        angles = np.deg2rad(angles)  # Перевод в радианы
        angle = self.find_largest_inf_gap(angles, ranges)
        largest_inf_gap_middle_deg = np.rad2deg(angle)
        rospy.loginfo(largest_inf_gap_middle_deg)
        left, right = self.calculate_wheel_speeds(largest_inf_gap_middle_deg, 25)
        self.left_wheel_publisher.publish(Float64(left))
        self.right_wheel_publisher.publish(Float64(right))


if __name__ == '__main__':
    rospy.init_node('robot_navigation')
    Robot()
    rospy.spin()
