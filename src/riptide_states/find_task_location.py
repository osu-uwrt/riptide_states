#!/usr/bin/env python3

from flexbe_core import EventState, Logger
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher


class FindTaskLocation(EventState):
	"""
	Returns the general position and orientation of the gate task

	<= Success						Coordinates are published.

    <= Failed                       Coordinates are not published.

	"""

	def __init__(self, target):
		"""Constructor"""
		super(FindTaskLocation, self).__init__(outcomes=['Success', 'Failed'], output_keys=['x','y','z','orientation'])
		self._topic = target
		self._sub = ProxySubscriberCached({self._topic: target})
		self._start_time = rospy.Time.now()
		self._timeout_temp = 1




	def callback(self,data,userdata):
		userdata.x = data.pose.x
		userdata.y = data.pose.y
		userdata.z = data.pose.z
		userdata.orientation = data.orientation
		return 'Success'



	def execute(self,userdata):
		msg = PoseWithCovarianceStamped()
		if self._sub.has_msg(self._topic):
			msg = self._sub.get_last_msg(self._topic)
			self._sub.remove_last_msg(self._topic)
		if self.callback(msg,userdata) == 'Success':
			return 'Success'
		if self._start_time - rospy.Time.now() > self._timeout:
			return 'Failed'

	def on_enter(self, userdata):
		self._timeout = rospy.Duration.from_sec(self._timeout_temp)