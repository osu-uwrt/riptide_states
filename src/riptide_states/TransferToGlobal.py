#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
from flexbe_core.proxy import ProxySubscriberCached
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
import tf2_geometry_msgs
import tf


class TransferToGlobal(EventState):
	"""
	Turns coordinates relative to the robot into coordinates relative to total world

	<= Success						Coordinates are published.

    <= Failed                       Coordinates are not published.

	"""


	def __init__(self,x,y,z,orientation):
		"""Constructor"""
		super(TransferToGlobal, self).__init__(outcomes=['Success'], output_keys=['x','y','z','orientation'])
		self.loc_topic = '/puddles/odometry/filtered'
		self.sub = ProxySubscriberCached({self.loc_topic: Odometry})
		self._pose = Pose()
		self._pose.position.x = x
		self._pose.position.y = y
		self._pose.position.z = z
		self._pose.orientation = orientation
		self._frame = "/puddles/base_link"
		self._start_time = rospy.Time.now()
		self._timeout_temp = 1

		self.tl = tf.TransformListener()

	def callback(self,userdata):
		#Changing the coordinate system into the viewpoint of the target to easily move the robot three feet in front of it
		
		
		t = self.tl.getLatestCommonTime(self._frame,"/world")
		pl = PoseStamped()
		pl.header.frame_id = self._frame
		pl.header.stamp = t
		pl.pose = self._pose
		convertedPos = self.tl.transformPose("/world", pl)

		#Logger.loginfo('XYZ: {},{},{} and orientation {},{},{},{}'.format(convertedPos.position.x,convertedPos.position.y,convertedPos.position.z,transformed_pose.position.orientation.x,transformed_pose.position.orientation.y,transformed_pose.position.orientation.z,transformed_pose.position.orientation.w))
		userdata.x = convertedPos.pose.position.x
		userdata.y = convertedPos.pose.position.y
		userdata.z = convertedPos.pose.position.z
		userdata.orientation = convertedPos.pose.orientation
		return 'Success'

		



	def execute(self,userdata):
		
			if self.callback(userdata) == 'Success':
				return 'Success'
			if self._start_time - rospy.Time.now() > self._timeout:
				return 'Failed'


	def on_enter(self, userdata):
		msg = tf2_geometry_msgs.PoseStamped()
		if self.sub.has_msg(self.loc_topic):
			msg = self.sub.get_last_msg(self.loc_topic)
			self.sub.remove_last_msg(self.loc_topic)
		if self._pose.orientation is None:
			self._pose.orientation = msg.pose.pose.orientation
		self._timeout = rospy.Duration.from_sec(self._timeout_temp)