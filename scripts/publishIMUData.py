#! /usr/bin/python3
import rospy
import depthai as dai
from localization.msg import accelero,gyro
from localization.msg import imum

pipeline = dai.Pipeline()
imu = pipeline.create(dai.node.IMU)
xlinkout = pipeline.create(dai.node.XLinkOut)
xlinkout.setStreamName("imu")
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW,500)
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW,400)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)
imu.out.link(xlinkout.input)
rospy.init_node("OakD_IMU")

accPub = rospy.Publisher("/oakd_accel",accelero,queue_size =1000)
gyroPub = rospy.Publisher("/oakd_gyro",gyro,queue_size =1000)
imuPub = rospy.Publisher("/oakd_imu",imum,queue_size =1000)
rate = rospy.Rate(1000)
with dai.Device(pipeline) as device:
    print("Device connected")
    imuQueue = device.getOutputQueue(name="imu",maxSize=50,blocking=False)
    while not rospy.is_shutdown():
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

            imuMsg.gyroX = gyroValues.x
            imuMsg.gyroY = gyroValues.y
            imuMsg.gyroZ = gyroValues.z
            imuMsg.accX = acceleroValues.x
            imuMsg.accY = acceleroValues.y
            imuMsg.accZ = acceleroValues.z

            imuPub.publish(imuMsg)
            
        rate.sleep()

