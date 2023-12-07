#! /usr/bin/python3
import rospy
import depthai as dai
from localization.msg import accelero,gyro
from localization.msg import imum
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation

pipeline = dai.Pipeline()
imu = pipeline.create(dai.node.IMU)
xlinkout = pipeline.create(dai.node.XLinkOut)
xlinkout.setStreamName("imu")
imu.enableIMUSensor(dai.IMUSensor.LINEAR_ACCELERATION,400)
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED,400)
imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR,400)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)
imu.out.link(xlinkout.input)
rospy.init_node("OakD_IMU")

accPub = rospy.Publisher("/oakd/accel",accelero,queue_size =1000)
gyroPub = rospy.Publisher("/oakd/gyro",gyro,queue_size =1000)
imuPub = rospy.Publisher("/oakd/imu",imum,queue_size =1000)
fullPub = rospy.Publisher("/imu/data",Imu,queue_size=1000)

#TODO getChipTemperature() getConnectedIMU() getAllConnectedDevices()
rate = rospy.Rate(1000)
with dai.Device(pipeline) as device:
    print("Device connected")
    imuQueue = device.getOutputQueue(name="imu",maxSize=50,blocking=False)
    while not rospy.is_shutdown():
        if device.getChipTemperature().average>=55: print("OAK-D temperature above 55, safety shutdown activated"); exit(0)
        accVal = accelero()
        gyroVal = gyro()
        imuMsg = imum()
        imuData = imuQueue.get()
        imuPackets = imuData.packets

        for imuPacket in imuPackets:
            gyroValues = imuPacket.gyroscope
            gyroVal.gyroX = gyroValues.x
            gyroVal.gyroY = gyroValues.y
            gyroVal.gyroZ = gyroValues.z
            gyroPub.publish(gyroVal)

            acceleroValues = imuPacket.acceleroMeter
            accVal.accX = acceleroValues.x
            accVal.accY = acceleroValues.y
            accVal.accZ = acceleroValues.z
            accPub.publish(accVal)

            rotVec = imuPacket.rotationVector

            imuMsg.gyroX = gyroValues.x
            imuMsg.gyroY = gyroValues.y
            imuMsg.gyroZ = gyroValues.z
            imuMsg.accX = acceleroValues.x
            imuMsg.accY = acceleroValues.y
            imuMsg.accZ = acceleroValues.z
    
            imuPub.publish(imuMsg)

            finImu = Imu()
            finImu.header.frame_id = "base_link"
            finImu.header.stamp.secs = int(rospy.get_time())
            finImu.orientation.x = rotVec.i
            finImu.orientation.y = rotVec.j
            finImu.orientation.z = rotVec.k
            finImu.orientation.w = rotVec.real
            finImu.linear_acceleration.x = acceleroValues.x
            finImu.linear_acceleration.y = acceleroValues.y
            finImu.linear_acceleration.z = acceleroValues.z
            finImu.angular_velocity.x = gyroValues.x
            finImu.angular_velocity.y = gyroValues.y
            finImu.angular_velocity.z = gyroValues.z
            for i in range(9):
                finImu.angular_velocity_covariance[i] = 0
                finImu.linear_acceleration_covariance[i] = 0
                finImu.orientation_covariance[i] = 0
            finImu.angular_velocity_covariance[0] = 1e-5
            finImu.angular_velocity_covariance[4] = 1e-5
            finImu.angular_velocity_covariance[8] = 1e-5
            finImu.linear_acceleration_covariance[0] = 1e-5
            finImu.linear_acceleration_covariance[4] = 1e-5
            finImu.linear_acceleration_covariance[8] = 1e-5
            finImu.orientation_covariance[0] = 1e-5
            finImu.orientation_covariance[4] = 1e-5
            finImu.orientation_covariance[8] = 1e-5
            fullPub.publish(finImu)
            orientation_array = [finImu.orientation.x,
                                 finImu.orientation.y,
                                 finImu.orientation.z,
                                 finImu.orientation.w]
            print(Rotation.from_quat(orientation_array).as_euler("xyz",degrees=True))
        rate.sleep()

