#!/usr/bin/env python3

from flexbe_core import EventState, Logger
import rospy
import riptide_controllers.msg
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import PoseStamped


class BigTranslateState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, topic):
		"""Constructor"""
		super(BigTranslateState, self).__init__(outcomes=['Success', 'Failure'],
            input_keys=['x','y'])
		self._topic = topic
		self.client = ProxyActionClient({self._topic: riptide_controllers.msg.MoveDistanceAction})
		#self._pub = ProxyPublisher({self._topic: PoseStamped})


	def execute(self, userdata):
		if self.client.has_result(self._topic):
			result = self.client.get_result(self._topic)
			status = 'Success'       
			return status
	
	def on_enter(self, userdata):
		Logger.loginfo('Translating by vector <%f, %f>'%(userdata.x, userdata.y))
		self.client.send_goal(self._topic, riptide_controllers.msg.MoveDistanceGoal(userdata.x, userdata.y))