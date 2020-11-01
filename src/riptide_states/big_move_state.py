#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
from flexbe_core.proxy import ProxyActionClient

from flexbe_core.proxy import ProxyPublisher
from geometry_msgs.msg import PoseStamped


class BigMoveState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, topic):
		"""Constructor"""
		super(BigMoveState, self).__init__(outcomes=['done'],
												input_keys=['pose'])

		self._topic = topic
		# self._pub = ProxyPublisher({self._topic: PoseStamped})
		self.client = ProxyActionClient({self._topic: PoseStamped})


	def execute(self, userdata):
		if self.client.has_result(self._topic):
    		result = self.client.get_result(self._topic)
			status = 'Success'       
			return status
	
	def on_enter(self, userdata):
		self._pub.publish(self._topic, userdata.pose)