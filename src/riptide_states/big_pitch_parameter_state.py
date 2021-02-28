#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import riptide_controllers.msg
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from flexbe_core.proxy import ProxySubscriberCached 
from tf import transformations
from tf.transformations import quarternion_multiply

class BigPitchParameterState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, topic, angle):
		"""Constructor"""
		super(BigPitchParameterState, self).__init__(outcomes=['Success', 'Failure'])
		self._topic = topic
		self._angle = angle
		self.client = ProxyPublisher({self._topic: PoseStamped})
		self._sub = ProxySubscriberCached({'/puddles/odometry/filtered': Odometry}) # outputs current state variables, gives q of orientation

	def execute(self, userdata):
		if self.client.has_result(self._topic):
			result = self.client.get_result(self._topic)
			status = 'Success'       
			return status
	
	def on_enter(self, userdata):
		Logger.loginfo('Pitching with angle %f'%self._angle)
		radian = math.radians(self._angle) # converts angle from degrees to radians
		quarternion = quaternion_from_euler(0,0,radian) # converts from radians to quaternions
		initmsg = self._sub.get_last_msg('/puddles/odometry/filtered') # initial position
		self._sub.remove_last_msg('/puddles/odometry/filtered') # clear
		
		initOrientation = initmsg.pose.pose.orientation 
		newQuat = quarternion_multiply(initOrientation, createdQuaternion)
		self.client.send_goal(self._topic, riptide_controllers.msg.GoToPitchGoal(self._angle))