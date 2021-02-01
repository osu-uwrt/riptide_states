#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher


class FindTask(EventState):
	"""
	Returns the general position and orientation of the gate task

	<= Success						Coordinates are published.

    <= Failed                       Coordinates are not published.

	"""

	def __init__(self, target, subscribe):
		"""Constructor"""
		super(FindTask, self).__init__(outcomes=['Success', 'Failed'], output_keys=['position','orientation'])
		self._sub = ProxySubscriberCached({self._topic: subscribe})
		self._start_time = rospy.Time.now()
		self._timeout_temp = 1




	def callback(self,bbox,userdata):
		userdata.position = self._sub.pose
		userdata.orientation = self._sub.orientation
		return 'Success'



	def execute(self,userdata):
		msg = FindTask()
		if self._sub.has_msg(self._topic):
			msg = self._sub.get_last_msg(self._topic)
			self._sub.remove_last_msg(self._topic)
		if self.callback(msg,userdata) == 'Success':
			return 'Success'
		if self._start_time - rospy.Time.now() > self._timeout:
			return 'Failed'

	def on_enter(self, userdata):
		self._timeout = rospy.Duration.from_sec(self._timeout_temp)