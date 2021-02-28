#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import riptide_controllers.msg
import math
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyActionClient
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


class BigYawParameterState(EventState):
	"""
	This state changes the yaw of the robot using parameters in Flexbe.

	-- topic 		string 			Topic for the pitch action controller

	-- angle		numeric			Angle to change to

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, topic, angle):
		"""Constructor"""
		super(BigYawParameterState, self).__init__(outcomes=['Success', 'Failure'])
		self._topic = topic
		self._angle = angle
		self._pub = ProxyPublisher({self._topic: PoseStamped})
		self._sub = ProxySubscriberCached({'/puddles/odometry/filtered': Odometry})


	def execute(self, userdata):
		#if self.client.has_result(self._topic):
			#result = self.client.get_result(self._topic)
			#status = 'Success'       
			#return status
		if self._sub.has_msg('/puddles/odometry/filtered'):
			msg = self._sub.get_last_msg('/puddles/odometry/filtered')
			currentOrientation = msg.pose.pose.orientation


	
	def on_enter(self, userdata):
		Logger.loginfo('Yawing with angle %f'%self._angle)
		radian = math.radians(self._angle)
		q = quaternion_from_euler(0,0,radian)
		msg = PoseStamped()
		msg.pose.orientation.x = q.x
		self._pub.publish(self._topic, q)
		
		