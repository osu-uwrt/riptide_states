#!/usr/bin/env python3

from flexbe_core import EventState, Logger
import rospy
import riptide_controllers.msg
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import PoseStamped


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
		self.client = ProxyActionClient({self._topic: riptide_controllers.msg.GoToYawAction})
		#self._pub = ProxyPublisher({self._topic: PoseStamped})


	def execute(self, userdata):
		if self.client.has_result(self._topic):
			result = self.client.get_result(self._topic)
			status = 'Success'       
			return status
	
	def on_enter(self, userdata):
		Logger.loginfo('Yawing with angle %f'%self._angle)
		self.client.send_goal(self._topic, riptide_controllers.msg.GoToYawGoal(self._angle))