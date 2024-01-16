#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float64
import sensor_msgs.point_cloud2 as pc2
import message_filters
import numpy as np

control_speed = 5
def calculate_wheel_speeds(target_angle, max_wheel_speed):
    global control_speed

    target_angle_rad = np.deg2rad(target_angle)
    if  not 86 <= target_angle <= 95:
        if control_speed-0.1 > 5:
            control_speed -= 0.1
    else:
        if max_wheel_speed > control_speed:
                    control_speed = min( max_wheel_speed, control_speed + 0.02 )
        elif max_wheel_speed < control_speed:
                    control_speed = max( max_wheel_speed, control_speed - 0.02 )
        else:
                    control_speed = max_wheel_speed
    if target_angle_rad < np.pi / 2:
        left_wheel_speed = control_speed * (1 - (target_angle_rad - np.pi / 2) / (np.pi / 2))
        right_wheel_speed = control_speed
    else:
        left_wheel_speed = control_speed
        right_wheel_speed = control_speed * (1 - (np.pi - target_angle_rad - np.pi / 2) / (np.pi / 2))
    rospy.loginfo("%s", control_speed)
    return left_wheel_speed, right_wheel_speed

def find_largest_inf_gap(angles, values):
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
def lidar_callback(data):
        # Преобразование данных лидара в массив расстояний
        ranges = data.ranges
        # rospy.loginfo(ranges)
        angles = 20+ np.arange(len(ranges)) * 2.8
        angles = np.deg2rad(angles)  # Перевод в радианы
        angle = find_largest_inf_gap(angles, ranges)
        largest_inf_gap_middle_deg = np.rad2deg(angle)
        rospy.loginfo(largest_inf_gap_middle_deg)
        left, right = calculate_wheel_speeds(largest_inf_gap_middle_deg, 25)
        left_wheel_publisher.publish(Float64(left))
        right_wheel_publisher.publish(Float64(right))
min_gap = 0.4
wheel_pub = 0
left_wheel_publisher = 0 
right_wheel_publisher = 0
def sensor_callback(front_lidar):
    lidar_callback(front_lidar)
if __name__ == '__main__':
    rospy.init_node('robot_navigation')
    front_lidar = message_filters.Subscriber('diff_drive_robot/laser/scan', LaserScan)
    left_wheel_publisher = rospy.Publisher("/robot_control/left_wheel_joint/command", Float64, queue_size=10)
    right_wheel_publisher = rospy.Publisher("/robot_control/right_wheel_joint/command", Float64, queue_size=10)
    ts = message_filters.ApproximateTimeSynchronizer([front_lidar], 10, 0.1)
    ts.registerCallback(sensor_callback)
    rospy.spin()
