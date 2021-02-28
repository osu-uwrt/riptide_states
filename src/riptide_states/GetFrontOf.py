#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher
from geometry_msgs.msg import Pose
import tf2_geometry_msgs
import tf2_ros


class GetFrontOf(EventState):
	"""
	Changes the input position to be 3 feet in front of the object.

	<= Success						Coordinates are published.

    <= Failed                       Coordinates are not published.

	"""

	def transform_pose(self, input_pose, from_frame, to_frame):

		# **Assuming /tf2 topic is being broadcasted
		tf_buffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(tf_buffer)
		pose_stamped = tf2_geometry_msgs.PoseStamped()
		pose_stamped.pose = input_pose
		pose_stamped.header.frame_id = from_frame
		pose_stamped.header.stamp = rospy.Time.now()
		try:
			# ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
			output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
			return output_pose_stamped.pose
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			raise



	def __init__(self,target):
		"""Constructor"""
		super(GetFrontOf, self).__init__(outcomes=['Success'], output_keys=['x','y','z','orientation'])
		self._frame = target
		self._start_time = rospy.Time.now()
		self._timeout_temp = 1


	def callback(self,userdata):
		_pose = Pose()
		#Changing the coordinate system into the viewpoint of the target to easily move the robot three feet in front of it
		transformed_pose = transform_pose(_pose, "world", self._frame)
		transformed_pose.position.x = -3
		transformed_pose.position.y = 0
		transformed_pose.position.z = 0
		transformed_pose.orientation.x = 0
		transformed_pose.orientation.y = 0
		transformed_pose.orientation.z = 0
		transformed_pose.orientation.w = 0
		#Changing the coordinates back into global
		_updated_pose = transform_pose(transformed_pose,self._frame,"world")
		#Splitting the pose to be able to be used in the move state (Maybe change move state to use pose instead of xyzw)
		userdata.x = _updated_pose.x
		userdata.y = _updated_pose.y
		userdata.z = _updated_pose.z
		userdata.orientation = _updated_pose.orientation
		return 'Success'

		



	def execute(self,userdata):
			if self.callback(userdata) == 'Success':
				return 'Success'
			if self._start_time - rospy.Time.now() > self._timeout:
				return 'Failed'


	def on_enter(self, userdata):
		self._timeout = rospy.Duration.from_sec(self._timeout_temp)