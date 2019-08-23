#!/usr/bin/python
import roslib
roslib.load_manifest("moby")
import rospy, time, sys, navio.mpu9250, std_msgs.msg
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3
from moby.msg import Yaw


def imu_pub():
    rate = rospy.Rate(100)
    no_covariance_matrix = (0, 0, 0, 0, 0, 0, 0, 0, 0)
    unknown_matrix = (-1, 0, 0, 0, 0, 0, 0, 0, 0)
    unknown_quat = Quaternion(0, 0, 0, 0)
    
    yaw = 0

    imu = navio.mpu9250.MPU9250()
    if imu.testConnection():
        print "Connection OK"
    else:
        sys.exit("Connection failed")
        
    accel_gyro_pub = rospy.Publisher("imu/data_raw", Imu)
    mag_pub = rospy.Publisher("imu/mag", MagneticField)
    yaw_pub = rospy.Publisher("yaw", Yaw)
    
    imu.initialize()
    
    rospy.sleep(1.0)
    
    last_time = rospy.get_time()
    
    while not rospy.is_shutdown():
        m9a, m9g, m9m = imu.getMotion9()
        
        m9m_corrected = (m9m[1], m9m[0], -m9m[2]) # the magnetometer is oriented differently in the package
        
        accel_vec = Vector3(*m9a)
        angvel_vec = Vector3(-m9g[0], -m9g[1], m9g[2])
        mag_vec = Vector3(*m9m_corrected)
        
        accel_gyro_header = std_msgs.msg.Header()
        accel_gyro_header.stamp = rospy.Time.now()
        accel_gyro_header.frame_id = "base_link"
        
        mag_header = std_msgs.msg.Header()
        mag_header.stamp = rospy.Time.now()
        mag_header.frame_id = "base_link"
        
        current_time = rospy.get_time()
        yaw -= m9g[2] * (current_time - last_time)
        last_time = current_time
        
        accel_gyro_pub.publish(Imu(accel_gyro_header, unknown_quat, unknown_matrix, angvel_vec, no_covariance_matrix, accel_vec, no_covariance_matrix))
        mag_pub.publish(MagneticField(mag_header, mag_vec, no_covariance_matrix))
        yaw_pub.publish(Yaw(yaw))
        
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('imu_pub')
    try:
        imu_pub()
    except rospy.ROSInterruptException: 
        pass