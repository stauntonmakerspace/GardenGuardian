#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

from ackermann_msgs.msg import AckermannDriveStamped
from numpy import sin, cos, tan

def tck_callback(data):
	global last_time
	global x_
	global y_
	global yaw_

	current_time = data.header.stamp
	delta_time = (current_time - last_time).to_sec() # seconds

	#basic velocity inputs
	current_speed = data.drive.speed #m/s from rear wheels
	steering_angle = data.drive.steering_angle #radians

	#compute odometry values from joint angles
	#and get the theta update
	angular_velocity = current_speed * tan(steering_angle) / wheelbase # rad 
	
	#compute odometry update values
	delta_x = current_speed * cos(yaw_)
	delta_y = current_speed * sin(yaw_)

	#now update our pose estimate
	x_ += delta_x * delta_time
	y_ += delta_y * delta_time

	yaw_ += angular_velocity * delta_time
	
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw_)

	# first, we'll publish the transform over tf
	odom_broadcaster.sendTransform(
		(x_, y_, 0.),
		odom_quat,
		current_time,
		"base_link",
		"odom"
	)

	#now update our pose estimate
	odom = Odometry()
 	odom.header.stamp = current_time
        odom.header.frame_id = "odom"
	odom.pose.pose = Pose(Point(x_, y_, 0.), Quaternion(*odom_quat))

	# Co variance should be cacluated empirically
	odom.pose.covariance[0]  = 0.2 #< x
	odom.pose.covariance[7]  = 0.2 #< y
	odom.pose.covariance[35] = 0.4 #< yaw

	# Set Velocity
	odom.child_frame_id = 'base_link'
        odom.twist.twist = Twist(Vector3(current_speed, 0, 0), Vector3(0, 0, angular_velocity))
	pub.publish(odom)
	last_time = current_time

if __name__ == '__main__': 
  try:    
    rospy.init_node('rc_odom_node')
    odom_broadcaster = tf.TransformBroadcaster()
    last_time = rospy.Time.now()
    x_ = 0
    y_ = 0
    yaw_ = 0
    
    # twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    # ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
    # frame_id = rospy.get_param('~frame_id', 'odom')

    wheelbase = rospy.get_param('~wheelbase', 10.0)
    
    rospy.Subscriber("/rc_state", AckermannDriveStamped, tck_callback) #, queue_size=1)
    pub = rospy.Publisher("odom", Odometry, queue_size=5)

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
