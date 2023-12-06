#! /usr/bin/python3
import rospy
from localization.msg import imum
from sensor_msgs.msg import Imu

rospy.init_node("process_IMU")
pubber = rospy.Publisher("/imu/data",Imu,queue_size=1000)
def cb(obj):
    accX = obj.accX
    accY = obj.accY
    accZ = obj.accZ

    gyroX = obj.gyroX
    gyroY = obj.gyroY
    gyroZ = obj.gyroZ

    imuData = Imu()
    imuData.header.frame_id = "base_link"
    imuData.orientation.x = imuData.orientation.y = imuData.orientation.z = imuData.orientation.w = 0
    imuData.orientation_covariance[0]=-1
    
    imuData.angular_velocity.x = gyroX
    imuData.angular_velocity.y = gyroY
    imuData.angular_velocity.z = gyroZ
    for i in range(9):
        imuData.angular_velocity_covariance[i] = 0

    imuData.linear_acceleration.x = accX
    imuData.linear_acceleration.y = accY
    imuData.linear_acceleration.z = accZ
    for i in range(9):
        imuData.linear_acceleration_covariance[i] = 0
    imuData.angular_velocity_covariance[0] = 1
    imuData.angular_velocity_covariance[3] = 1
    imuData.angular_velocity_covariance[6] = 1
    imuData.linear_acceleration_covariance[0] = 1
    imuData.linear_acceleration_covariance[3] = 1
    imuData.linear_acceleration_covariance[6] = 1
    pubber.publish(imuData)
    





print("Processing initiated")
rospy.Subscriber("/oakd_imu",imum,cb)
rospy.spin()