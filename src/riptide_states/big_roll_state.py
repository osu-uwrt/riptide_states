#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import riptide_controllers.msg
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import PoseStamped


class BigRollState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, topic):
		"""Constructor"""
		super(BigRollState, self).__init__(outcomes=['Success', 'Failure'],
            input_keys=['angle'])
		self._topic = topic
		self.client = ProxyActionClient({self._topic: riptide_controllers.msg.GoToRollAction})
		#self._pub = ProxyPublisher({self._topic: PoseStamped})


	def execute(self, userdata):
		if self.client.has_result(self._topic):
			result = self.client.get_result(self._topic)
			status = 'Success'       
			return status
	
	def on_enter(self, userdata):
		Logger.loginfo('Rolling with angle %f'%userdata.angle)
		self.client.send_goal(self._topic, riptide_controllers.msg.GoToRollGoal(userdata.angle))