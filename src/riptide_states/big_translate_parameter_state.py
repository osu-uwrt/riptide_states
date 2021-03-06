#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import riptide_controllers.msg
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import PoseStamped


class BigTranslateParameterState(EventState):
	"""
	Changes the absolute position of the robot on the xy plane

	-- topic 		string 			Topic to which the pose will be published.

	-- x			numeric			The x value to translate

	-- y			numeric			The y value to translate

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, topic, x, y):
		"""Constructor"""
		super(BigTranslateParameterState, self).__init__(outcomes=['Success', 'Failure'])
		self._topic = topic
		self._x = x
		self._y = y
		self.client = ProxyActionClient({self._topic: riptide_controllers.msg.MoveDistanceAction})
		#self._pub = ProxyPublisher({self._topic: PoseStamped})


	def execute(self, userdata):
		if self.client.has_result(self._topic):
			result = self.client.get_result(self._topic)
			status = 'Success'       
			return status
	
	def on_enter(self, userdata):
		Logger.loginfo('Translating by vector <%f, %f>'%(self._x, self._y))
		self.client.send_goal(self._topic, riptide_controllers.msg.MoveDistanceGoal(self._x, self._y))