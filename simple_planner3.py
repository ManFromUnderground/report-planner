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

ball = SphereParams()
#based_ball = tf2_geometry_msgs.PointStamped()

def get_ball(param):
	global ball
	#global based_ball
	point1 = tf2_geometry_msgs.PointStamped()
	ball.xc = param.xc
	ball.yc = param.yc
	ball.zc = param.zc
	ball.radius = param.radius
	"""if param.xc != 0.0 and param.yc != 0.0 and param.zc != 0.0:
		point1.point.x = param.xc
		point1.point.y = param.yc
		point1.point.z = param.zc
		point1.header.stamp = 'camera_color_optical'
	buffer2 = tf2_ros.Buffer()
	listener2 = tf2_ros.TransformListener(buffer2)
	if param.xc != 0.0 and param.yc != 0.0 and param.zc != 0.0:
		based_ball = buffer2.transform(point1,'base',rospy.Duration(1.0))"""
	

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
	
	buffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(buffer)
	print("listener:", listener)
	qrot = Quaternion()
	
	
	cam = tf2_geometry_msgs.PointStamped()
	
	base = tf2_geometry_msgs.PointStamped()
	#cam.x, cam.y, cam.z, cam.rotation = None, None, None, None
	
	while ((cam.point.x is None or ball.xc == 0.0) or (cam.point.y is None or ball.yc == 0.0) or (cam.point.z is None or ball.zc == 0.0)) or not_except == True:
		try:
			trans = buffer.lookup_transform("base", "camera_color_optical", rospy.Time())
			print("trans is:", trans)
			not_except = True
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('frames unavailable')
			print("cam x:", cam.point.x)
			print("cam y:", cam.point.y)
			print("cam z:", cam.point.z)
			print("ball x:", ball.xc)
			print("ball y:", ball.yc)
			print("ball z:", ball.zc)
			
			continue
		"""x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z"""
		
		"""qrot = trans.transform.rotation
		roll, pitch, yaw, = euler_from_quaternion([qrot.x, qrot.y, qrot.z, qrot.w])"""
		cam.header.frame_id = "camera_color_optical"
		cam.header.stamp = rospy.get_rostime()
		#while ball.xc != cam.point.x:
		cam.point.x = ball.xc
		cam.point.y = ball.yc
		cam.point.z = ball.zc
			
		print("attempted to assign")
		
		base = buffer.transform(cam, 'base', rospy.Duration(1.0))
		
		qrot = cam.transform.rotation
		roll, pitch, yaw, = euler_from_quaternion([qrot.x, qrot.y, qrot.z, qrot.w])
		loop_rate.sleep()
		
	# define a plan variable
	plan = Plan()
	
	plan_point0 = Twist()
	
	
	# above ball
	plan_point0.linear.x = base.point.x
	plan_point0.linear.y = base.point.y
	plan_point0.linear.z = base.point.z + 0.5
	plan_point0.angular.x = 1.57
	plan_point0.angular.y = 0.0
	plan_point0.angular.z = 0.0
	# add this point to the plan
	plan.points.append(plan_point0)
	
	print("plan 0:", plan_point0)
	
	plan_point1 = Twist()
	plan_point1.linear.x = base.point.x
	plan_point1.linear.y = base.point.y
	plan_point1.linear.z = base.point.z
	plan_point1.angular.x = 1.57
	plan_point1.angular.y = 0.0
	plan_point1.angular.z = 0.0
	# add this point to the plan
	plan.points.append(plan_point1)
	
	print(plan_point1)
	
	plan_point2 = Twist()
	# define a point away from the initial position
	plan_point2.linear.x = -0.672
	plan_point2.linear.y = -0.233
	plan_point2.linear.z = 0.124
	plan_point2.angular.x = 1.57
	plan_point2.angular.y = -0.45
	plan_point2.angular.z = 0.0
	# add this point to the plan
	plan.points.append(plan_point2)
	
	plan_point3 = Twist()
	# define a another point away from the initial position
	plan_point3.linear.x = -0.32
	plan_point3.linear.y = -0.62
	plan_point3.linear.z = 0.29
	plan_point3.angular.x = 1.57
	plan_point3.angular.y = -0.2
	plan_point3.angular.z = 0.75
	# add this point to the plan
	plan.points.append(plan_point3)
	
	plan_point4 = Twist()
	# set object down
	plan_point4.linear.x = -0.672
	plan_point4.linear.y = -0.233
	plan_point4.linear.z = 0.124
	plan_point4.angular.x = 1.57
	plan_point4.angular.y = -0.2
	plan_point4.angular.z = 0.75
	# add this point to the plan
	plan.points.append(plan_point4)

	
	print("reached last while")
	while not rospy.is_shutdown():
		
		
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()

#use plan points to compare the location of the machine with the intended target
