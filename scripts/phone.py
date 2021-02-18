#!/usr/bin/env python
# Author: Sung Jik Cha
# Modified by: Nile Walker
# Credits: ros turtlebot node: https://github.com/Arkapravo/turtlebot
import rospy
import socket
import time 
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField

def imu_publisher(UDP_IP,UDP_PORT,BUFFER_SIZE=1024,debug = False):
    GYRO_X_OFFSET = 0.0
    GYRO_Y_OFFSET = 0.0
    GYRO_Z_OFFSET = 0.0
    pub_freq = 10
    iter_count = 0
    num_callibration_itrs = 60

    gps_pub = rospy.Publisher('phone/fix', NavSatFix, queue_size=50)
    imu_pub = rospy.Publisher('phone/imu', Imu, queue_size=50)
    mag_pub = rospy.Publisher('phone/magnetic_field', MagneticField, queue_size=50)

    rospy.init_node('imu_publisher', anonymous=True)
    rate = rospy.Rate(pub_freq)

    rospy.loginfo("waiting for device...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    connected = False

    while not rospy.is_shutdown():
        if not connected:
            try:
                sock.bind((UDP_IP, UDP_PORT))
                connected = True 
                rospy.loginfo("Device Connected @ {}:{}".format(UDP_IP,UDP_PORT))
            except:
                rospy.loginfo("{0} UDP link failed. Retrying every second...".format(UDP_IP))
                time.sleep(1)
                continue

        data,_ = sock.recvfrom(1024)
        line = data.split(',')
        if len(line) == 12:  #received complete packet

            acceleration_x = float(line[0])
            acceleration_y = float(line[1])
            acceleration_z = float(line[2])

            magnetic_x = float(line[3])
            magnetic_y = float(line[4])
            magnetic_z = float(line[5])

            gyro_x = float(line[6])
            gyro_y = float(line[7])
            gyro_z = float(line[8])

            lat = float(line[-3])
            long = float(line[-2])
            alt = float(line[-1][:-1])
            
            if iter_count < num_callibration_itrs:
                GYRO_X_OFFSET += gyro_x
                GYRO_Y_OFFSET += gyro_y
                GYRO_Z_OFFSET += gyro_z
                iter_count += 1
            elif iter_count == num_callibration_itrs and num_callibration_itrs != 0:
                GYRO_X_OFFSET /= num_callibration_itrs
                GYRO_Y_OFFSET /= num_callibration_itrs
                GYRO_Z_OFFSET /= num_callibration_itrs
                rospy.loginfo("finished callibrating yaw")
                iter_count += 1

            #Publish Ros Imu message
            else:
                gyro_x -= GYRO_X_OFFSET
                gyro_y -= GYRO_Y_OFFSET
                gyro_z -= GYRO_Z_OFFSET

                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = 'phone_link'

                imu_msg.angular_velocity.x = gyro_x
                imu_msg.angular_velocity.y = gyro_y
                imu_msg.angular_velocity.z = gyro_z

                imu_msg.linear_acceleration.x = acceleration_x
                imu_msg.linear_acceleration.y = acceleration_y
                imu_msg.linear_acceleration.z = acceleration_z
                
                imu_msg.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
                imu_msg.angular_velocity_covariance[0] = -1
                imu_msg.linear_acceleration_covariance[0] = -1
                imu_pub.publish(imu_msg)

                mag_msg = MagneticField()
                mag_msg.header.stamp = rospy.Time.now()
                mag_msg.header.frame_id = 'phone_link'

                mag_msg.magnetic_field.x = magnetic_x
                mag_msg.magnetic_field.y = magnetic_y
                mag_msg.magnetic_field.z = magnetic_z

                mag_pub.publish(mag_msg)               

                gps_msg = NavSatFix()
                gps_msg.header.stamp = rospy.Time.now()
                gps_msg.header.frame_id = 'phone_link'
                gps_msg.latitude = lat
                gps_msg.longitude = long
                gps_msg.altitude = alt
                gps_pub.publish(gps_msg)

            rate.sleep()
        else:
            rospy.loginfo("received incomplete TCP packet from android IMU")
            continue
             
if __name__ == '__main__':
    try:
        if rospy.has_param('~num_callibration_itrs'):
            num_callibration_itrs = rospy.get_param('~num_callibration_itrs')
        
        host = rospy.get_param('~host', '192.168.1.98')
        port = rospy.get_param('~port', 5000)
        imu_publisher(host,port,1024)
    except rospy.ROSInterruptException:
        pass
