#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import riptide_controllers.msg
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import PoseStamped


class BigDistanceState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, topic):
		"""Constructor"""
		super(BigDistanceState, self).__init__(outcomes=['Success', 'Failure'],
            input_keys=['object'], output_keys=['dist'])
		self._topic = "get_distance"
		self.client = ProxyActionClient({self._topic: riptide_controllers.msg.GetDistanceAction})
		#self._pub = ProxyPublisher({self._topic: PoseStamped})


	def execute(self, userdata):
		if self.client.has_result(self._topic):
			userdata.dist = client.get_result().distance
			status = 'Success'       
			return status
	
	def on_enter(self, userdata):
		Logger.loginfo('Moving %f m away from %s'%(2, userdata.object))
		self.client.send_goal(self._topic, riptide_controllers.msg.GetDistanceGoal(userdata.object))