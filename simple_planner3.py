#!/usr/bin/env python3

import rospy
import math
import tf2_ros
import tf2_geometry_msgs


# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams
from tf.transformations import *
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool

ball = SphereParams()
got_ball = False
go = Bool()
go = False


def execute(bool):
	global go
	go = bool

def get_twist(x, y, z, roll, pitch, yaw):
	twist1 = Twist()
	twist1.linear.x = x
	twist1.linear.y = y
	twist1.linear.z = z
	twist1.angular.x = roll
	twist1.angular.y = pitch
	twist1.angular.z = yaw
	return twist1


def get_ball(param):
	global ball
	global got_ball
	point1 = tf2_geometry_msgs.PointStamped()
	ball.xc = param.xc
	ball.yc = param.yc
	ball.zc = param.zc
	ball.radius = param.radius
	got_ball = True
	

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	#sub to filtered parameters
	
	not_except = False
	
	ball_sub = rospy.Subscriber('/sphere_params', SphereParams, get_ball)
	
	bool_sub = rospy.Subscriber('/std_msgs/Bool', Bool, execute)
	
	buffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(buffer)
	
	qrot = Quaternion()
	
	
	cam = tf2_geometry_msgs.PointStamped()
	
	base = tf2_geometry_msgs.PointStamped()
	
	
	while not rospy.is_shutdown():
		if got_ball:
			try:
				trans = buffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				print("frame unavailable")
				
				continue
			x = trans.transform.translation.x
			y = trans.transform.translation.y
			z = trans.transform.translation.z
			
			qrot = trans.transform.rotation
			roll, pitch, yaw, = euler_from_quaternion([qrot.x, qrot.y, qrot.z, qrot.w])
			
			
			cam.header.frame_id = "camera_color_optical_frame"
			cam.header.stamp = rospy.get_rostime()
			#while ball.xc != cam.point.x:
			cam.point.x = ball.xc
			cam.point.y = ball.yc
			cam.point.z = ball.zc
			
			
			
			base = buffer.transform(cam, 'base', rospy.Duration(1.0))
			print('Test point in the cam frame:  x= ', format(cam.point.x, '.3f'), '(m), y= ', format(cam.point.y, '.3f'), '(m), z= ', format(cam.point.z, '.3f'),'(m)')
			print('Transformed point in the base frame:  x= ', format(base.point.x, '.3f'), '(m), y= ', format(base.point.y, '.3f'), '(m), z= ', format(base.point.z, '.3f'),'(m)')
			print('-------------------------------------------------')
			
			
			# define a plan variable
			plan = Plan()
			
			
			# above ball
			twist1 = get_twist(base.point.x, base.point.y, base.point.z + ball.radius + 0.1, 1.57, 0.0, 0.0)
			# add this point to the plan
			plan.points.append(twist1)
			
			twist2 = get_twist(base.point.x, base.point.y, base.point.z + ball.radius, 1.57, 0.0, 0.0)
			# add this point to the plan
			plan.points.append(twist2)
			
			# go back up
			plan.points.append(twist1)
			
			twist3 = get_twist(base.point.x + .2, base.point.y - .05, base.point.z + ball.radius + 0.1, 1.57, -0.45, 0.0)
			# add this point to the plan
			plan.points.append(twist3)
			
			twist4 = get_twist(base.point.x + .2, base.point.y - .05, base.point.z + ball.radius, 1.57, -0.2, 0.75)
			# add this point to the plan
			plan.points.append(twist4)
			
			plan.points.append(twist3)
			
			
			if go:
				# publish the plan
				plan_pub.publish(plan)
				# wait for 0.1 seconds until the next loop and repeat
			else:
				print("waiting for go")
		loop_rate.sleep()

#use plan points to compare the location of the machine with the intended target
