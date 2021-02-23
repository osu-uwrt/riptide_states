#!/usr/bin/env python3

from flexbe_core import EventState, Logger
import rospy
import riptide_controllers.msg
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import PoseStamped


class BigRollParameterState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, topic, angle):
		"""Constructor"""
		super(BigRollParameterState, self).__init__(outcomes=['Success', 'Failure'])
		self._topic = topic
		self._angle = angle
		self.client = ProxyActionClient({self._topic: riptide_controllers.msg.GoToRollAction})
		#self._pub = ProxyPublisher({self._topic: PoseStamped})


	def execute(self, userdata):
		if self.client.has_result(self._topic):
			result = self.client.get_result(self._topic)
			status = 'Success'       
			return status
	
	def on_enter(self, userdata):
		Logger.loginfo('Rolling with angle %f'%self._angle)
		self.client.send_goal(self._topic, riptide_controllers.msg.GoToRollGoal(self._angle))