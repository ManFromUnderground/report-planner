#!/usr/bin/env python3

import rospy
import math

# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

ball = SphereParams()

def get_ball(param):
	global ball
	ball.xc = param.xc
	ball.yc = param.yc
	ball.zc = param.zc
	ball.radius = param.radius



if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	#sub to filtered parameters
	
	ball_sub = rospy.Subscriber("/sphere_params", SphereParams, get_ball)
	
	# define a plan variable
	plan = Plan()
	plan_point0 = Twist()
	# above ball
	plan_point0.linear.x = ball.xc
	plan_point0.linear.y =ball.yc + 0.1
	plan_point0.linear.z = ball.zc
	plan_point0.angular.x = 1.57
	plan_point0.angular.y = 0.0
	plan_point0.angular.z = 0.0
	
	plan_point1 = Twist()
	# define a point close to the initial position
	plan_point1.linear.x = -0.42
	plan_point1.linear.y = -0.23
	plan_point1.linear.z = 0.4879
	plan_point1.angular.x = 1.57
	plan_point1.angular.y = 0.0
	plan_point1.angular.z = 0.0
	# add this point to the plan
	plan.points.append(plan_point1)
	
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

	
	
	while not rospy.is_shutdown():
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
