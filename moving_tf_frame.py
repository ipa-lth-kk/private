#!/usr/bin/python

import rospy
import tf
import time
import numpy as np
from math import cos,sin,pi,sqrt
from operator import add
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from std_msgs.msg import Empty as EmptyTopic



class MovingTf(object):
	def __init__(self):
		self._time = 0
		self._isStart = False
		self.zero=0
		self.time2 = time.time()
		self.acc=0.01
		self.line_acc=0.000001

		self._pub_marker = rospy.Publisher("visualization_marker", Marker, latch = True) # latch makes sure that the last published message reaches its target
		self._broadcaster = tf.TransformBroadcaster()

		self.start_services()
		self.get_parameters()

		# remove old markers
		self._trace.action = Marker.DELETE
		self._pub_marker.publish(self._trace)

		self._trace.action = Marker.MODIFY





	def start_services(self):
		## LTH Services ###
		# Could be actions...
		self.s1 = rospy.Service('~start', Empty, self.handle_start)
		self.sub1 = rospy.Subscriber("targets_start", EmptyTopic, self.handle_start)

		self.s2 = rospy.Service('~stop', Empty, self.handle_stop)
		self.sub2 = rospy.Subscriber("targets_stop", EmptyTopic, self.handle_stop)
		
		self.s3 = rospy.Service('~reset', Empty, self.handle_reset)
		self.sub3 = rospy.Subscriber("targets_reset", EmptyTopic, self.handle_reset)



	def get_parameters(self):
		self.tf_parent 	= rospy.get_param("~tf_parent", "base_link")
		self.tf_child	= rospy.get_param("~tf_child", "moving_tf")

		self.movement_type = rospy.get_param("~movement_type", "spiral") #line, circle, spiral

		rx = rospy.get_param("~move_frame_roll"	, 0)
		ry = rospy.get_param("~move_frame_pitch", 0)
		rz = rospy.get_param("~move_frame_yaw"	, 0)
		self.tf_quaternion = tf.transformations.quaternion_from_euler(rx, ry, rz)	

		Rx = tf.transformations.rotation_matrix(rx, (1, 0, 0))
		Ry = tf.transformations.rotation_matrix(ry, (0, 1, 0))
		Rz = tf.transformations.rotation_matrix(rz, (0, 0, 1))
		self._R = tf.transformations.concatenate_matrices(Rz, Ry, Rx)

		## Line Parameters
		if self.movement_type == "line":
			x_l = rospy.get_param("~line_x_start", 0)
			y_l= rospy.get_param("~line_y_start", 0)
			z_l = rospy.get_param("~line_z_start", 0)
			self._tf_start_coordinates = [x_l, y_l, z_l]



			x_d = rospy.get_param("~line_x_direction", 0.1) 
			y_d = rospy.get_param("~line_y_direction", 0)
			z_d = rospy.get_param("~line_z_direction", 0)

			T = np.dot(self._R, [x_d, y_d, z_d, 1])

			self._movement_vector = [T[0], T[1], T[2]]


		## Hexagon Parameters
		if self.movement_type == "hexagon":
			x_h = x_s = rospy.get_param("~x_start", 0)
			y_h = rospy.get_param("~y_start", 0)
			z_h = rospy.get_param("~z_start", 0)
			self._tf_start_coordinates = [x_h, y_h, z_h]
			
		##Point Parameters
		if self.movement_type == ("points" or "point"):
			x_p = x_s = rospy.get_param("~point_x_start", 0)
			y_p = rospy.get_param("~point_y_start", 0)
			z_p = rospy.get_param("~point_z_start", 0)
			self._tf_start_coordinates = [x_p, y_p, z_p]		


		## Circle Parameters
		if self.movement_type == "circle":
			self._x_c = rospy.get_param("~x_center", 1)
			self._y_c = rospy.get_param("~y_center", 0)
			self._z_c = rospy.get_param("~z_center", 0)

			self._radius = rospy.get_param("~radius", 0.5)

			self._tf_start_coordinates = [self._radius, 0, 0]

		## Spiral Parameters
		if self.movement_type == "spiral":
			self._inside_out = rospy.get_param("~inside_out", True)

			self._x_c = rospy.get_param("~x_center", 1)
			self._y_c = rospy.get_param("~y_center", 0)
			self._z_c = rospy.get_param("~z_center", 0)

			self._max_radius 		= rospy.get_param("~max_radius", 0.5)
			self._spiral_branch_d 	= rospy.get_param("~spiral_branch_distance", 0.2)

			if self._inside_out:
				self._tf_start_coordinates = [self._x_c, self._y_c, self._z_c]
			else:
				self._tf_start_coordinates = [self._max_radius, 0, 0]

		self._tf_coordinates = self._tf_start_coordinates


		## Time Parameters
		self.rate = rospy.get_param("~time_steps_hz", 50) # 10 Hz
		self.stop_time = rospy.get_param("~stop_time", 0) # stop tf movement after ... seconds


		## Trace Parameters
		self.show_path = rospy.get_param("~show_path", True)

		self._trace = Marker()
		self._trace.header.frame_id = self.tf_parent
		self._trace.header.stamp = rospy.Time.now()
		self._trace.id = 0
		self._trace.type= Marker.LINE_STRIP
		self._trace.ns = self.tf_child

		'''
		self._trace.scale.x = rospy.get_param("~trace_scale_x", 0.01)
		self._trace.scale.y = rospy.get_param("~trace_scale_y", 0.01)
		self._trace.scale.z = rospy.get_param("~trace_scale_z", 0.01)
		'''

		scale = rospy.get_param("~trace_scale", 0.01)

		self._trace.scale.x = scale
		self._trace.scale.y = scale
		self._trace.scale.z = scale

		self._trace.color.r = rospy.get_param("~trace_color_r", 1)
		self._trace.color.g = rospy.get_param("~trace_color_g", 0)
		self._trace.color.b = rospy.get_param("~trace_color_b", 0)
		self._trace.color.a = rospy.get_param("~trace_color_a", 1)



	def loop(self):
		rate = rospy.Rate(self.rate)

		while not rospy.is_shutdown():

			if self.stop_time: # != 0
				if self._time >= self.stop_time:
					self._isStart = False
			if self._isStart:
				if self.movement_type == "line":
					self.move_tf_linear()

				
				if self.movement_type == "hexagon":
					self.move_tf_hexagon()

				if self.movement_type == "points":
					self.move_tf_points()

				if self.movement_type == "point":
					self.move_tf_point()

				if self.movement_type == "circle":
					self.move_tf_circle(self._time)

				if self.movement_type == "spiral":
					self.move_tf_spiral(self._time)
				
				self._time += 1./self.rate

				if self.show_path:
					self._trace.points.append(Point(*self._tf_coordinates))
					if len (self._trace.points) > 0:
						self._pub_marker.publish(self._trace)

			#print self._tf_coordinates
			#print self.tf_quaternion

			self._broadcaster.sendTransform(
					self._tf_coordinates,
					self.tf_quaternion,
					rospy.Time.now(),
					self.tf_child,
					self.tf_parent)

			rate.sleep()


	def handle_start(self, req):
		self._isStart = True
		self._trace.action = Marker.MODIFY
		print "start"
		return []


	def handle_stop(self, req):
		self._isStart = False
		print "stop"
		return []


	def handle_reset(self, req):
		self._tf_coordinates = self._tf_start_coordinates
		self._time = 0
		self._trace.points = []
		self._trace.action = Marker.DELETE
		self._pub_marker.publish(self._trace)

		self.get_parameters()
		print "reset"
		return []


	def move_tf_linear(self):
		self._tf_coordinates = map(add, self._tf_coordinates, [self.line_acc*self._movement_vector[0], self.line_acc*self._movement_vector[1], self.line_acc*self._movement_vector[2]])
		#print self._tf_coordinates
		#self._tf_coordinates += self._movement_vector

		if self._tf_coordinates[0] > 1 or self._tf_coordinates[1] > 1 or self._tf_coordinates[2] > 1:
			self._isStart = False 
			return

		self.line_acc=self.line_acc+0.0001

	def move_tf_hexagon(self):
		#hexagon function
		if self.zero%6==0:
			self._tf_coordinates=[self._tf_coordinates[0]+0.1, self._tf_coordinates[1]+0.1732, self._tf_coordinates[2]]
			time.sleep(2)
		elif self.zero%6==1:
			self._tf_coordinates=[self._tf_coordinates[0]+0.2, self._tf_coordinates[1], self._tf_coordinates[2]]
			time.sleep(2)
		elif self.zero%6==2:
			self._tf_coordinates=[self._tf_coordinates[0]+0.1, self._tf_coordinates[1]-0.1732, self._tf_coordinates[2]]
			time.sleep(2)
		elif self.zero%6==3:
			self._tf_coordinates=[self._tf_coordinates[0]-0.1, self._tf_coordinates[1]-0.1732, self._tf_coordinates[2]]
			time.sleep(2)
		elif self.zero%6==4:
			self._tf_coordinates=[self._tf_coordinates[0]-0.2, self._tf_coordinates[1], self._tf_coordinates[2]]
			time.sleep(2)
		elif self.zero%6==5:
			self._tf_coordinates=[self._tf_coordinates[0]-0.1, self._tf_coordinates[1]+0.1732, self._tf_coordinates[2]]
			time.sleep(2)	
		self.zero=1+ self.zero



	def move_tf_points(self):
		if self.zero%2==0 and self.time2 + 5 < time.time():
			self._tf_coordinates=[self._tf_coordinates[0]-.2475, self._tf_coordinates[1]+0.47631, self._tf_coordinates[2]]
			#time.sleep(15)
			self.time2 = time.time()
			self.zero=1+ self.zero
		elif self.zero%2==1 and self.time2 + 5 < time.time():
			self._tf_coordinates=[self._tf_coordinates[0]+.2475, self._tf_coordinates[1]-0.47631, self._tf_coordinates[2]]
			#time.sleep(15)   
			self.time2 = time.time()
			self.zero=1+ self.zero



	def move_tf_point(self):
		pass



	def move_tf_circle(self, time):
		#circle function
		circle_x = self._radius * cos(time*self.acc)
		circle_y = self._radius * sin(time*self.acc)

		#rotate it
		T = np.dot(self._R, [circle_x, circle_y, 0, 1])

		#include x,y,z offset
		self._tf_coordinates = [self._x_c + T[0], self._y_c + T[1], self._z_c + T[2]]

		#increase acc
		self.acc=self.acc+0.0001


	def move_tf_spiral(self, time):
		#spiral function
		f_t = time * self._spiral_branch_d / (2 * pi)

		if f_t > self._max_radius:
			self._isStart = False
			return

		if self._inside_out:
			circle_x = f_t * cos(time)
			circle_y = f_t * sin(time)
		else:
			circle_x = (self._max_radius - f_t) * cos(time)
			circle_y = (self._max_radius - f_t) * sin(time)

		#rotate it
		T = np.dot(self._R, [circle_x, circle_y, 0, 1])

		#include x,y,z offset
		self._tf_coordinates = [self._x_c + T[0], self._y_c + T[1], self._z_c + T[2]]		


if __name__ == '__main__':
	rospy.init_node("moving_tf")#, anonymous=True)
	moving_tf = MovingTf()
	moving_tf.loop()
