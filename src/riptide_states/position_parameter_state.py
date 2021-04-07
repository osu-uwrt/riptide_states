#!/usr/bin/env python3

from flexbe_core import EventState, Logger
import rospy

from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import PoseStamped


class PositionParameterState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, x, y, z):
		"""Constructor"""
		super(PositionParameterState, self).__init__(outcomes=['Success', 'Failure'], output_keys = ['x','y','z','orientation'])
		self._x = x
		self._y = y
		self._z = z
		


	def execute(self, userdata):
		return 'Success'
	def on_enter(self, userdata):
		userdata.x = self._x
		userdata.y = self._y
		userdata.z = self._z
		userdata.orientation = None