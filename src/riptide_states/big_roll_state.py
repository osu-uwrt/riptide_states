#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import math
import riptide_controllers.msg
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from flexbe_core.proxy import ProxySubscriberCached 
from tf import transformations
from tf.transformations import quaternion_multiply

class BigRollState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, angle):
		"""Constructor"""
		super(BigRollState, self).__init__(outcomes=['Success', 'Failure'])
		self._topic = "/puddles/orientation"
		self._angle = angle
		self.client = ProxyPublisher({self._topic: PoseStamped})
		self._sub = ProxySubscriberCached({'/puddles/odometry/filtered': Odometry}) # outputs current state variables, gives q of orientation

	def execute(self, userdata):
		if self.client.has_result(self._topic):
			result = self.client.get_result(self._topic)
			status = 'Success'       
			return status
	
	def on_enter(self, userdata):
		Logger.loginfo('Rolling with angle %f'%self._angle)
		radian = math.radians(self._angle) # converts angle from degrees to radians
		myQuaternion = quaternion_from_euler(radian,0,0) # converts from radians to quaternions
		initmsg = self._sub.get_last_msg('/puddles/odometry/filtered') # initial position
		self._sub.remove_last_msg('/puddles/odometry/filtered') # clear
		
		initOrientation = initmsg.pose.pose.orientation 
		newQuat = quaternion_multiply(initOrientation, myQuaternion)
		self.client.publish(self._topic, newQuat)