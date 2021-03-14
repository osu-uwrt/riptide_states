#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
import tf


class GetFrontOf(EventState):
	"""
	Changes the input position to be 3 feet in front of the object.

	<= Success						Coordinates are published.

    <= Failed                       Coordinates are not published.

	"""

	def transform_pose(self, input_pose, from_frame, to_frame):
		tl = tf.TransformListener()
		t = tl.getLatestCommonTime(from_frame, to_frame)
		p1 = PoseStamped()
		p1.header.frame_id = from_frame
		p1.header.stamp = t
		p1.pose = input_pose
		convertedPos = tl.transformPose(to_frame, p1)
		return convertedPos


	def __init__(self,target):
		"""Constructor"""
		super(GetFrontOf, self).__init__(outcomes=['Success'], output_keys=['x','y','z','orientation'])
		self._frame = target
		self._start_time = rospy.Time.now()
		self._timeout_temp = 1


	def callback(self,userdata):

		#Changing the coordinate system into the viewpoint of the target to easily move the robot three feet in front of it
		transformed_pose = Pose()
		transformed_pose.position.x = -3
		transformed_pose.position.y = 0
		transformed_pose.position.z = 0
		transformed_pose.orientation.x = 0
		transformed_pose.orientation.y = 0
		transformed_pose.orientation.z = 0
		transformed_pose.orientation.w = 1
		#Changing the coordinates back into global
		_updated_pose = self.transform_pose(transformed_pose,self._frame,"world")
		#Splitting the pose to be able to be used in the move state (Maybe change move state to use pose instead of xyzw)
		userdata.x = _updated_pose.pose.position.x
		userdata.y = _updated_pose.pose.position.y
		userdata.z = _updated_pose.pose.position.z
		userdata.orientation = _updated_pose.pose.orientation
		return 'Success'

		



	def execute(self,userdata):
			if self.callback(userdata) == 'Success':
				return 'Success'
			if self._start_time - rospy.Time.now() > self._timeout:
				return 'Failed'


	def on_enter(self, userdata):
		self._timeout = rospy.Duration.from_sec(self._timeout_temp)