#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
from flexbe_core.proxy import ProxySubscriberCached
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf2_geometry_msgs
import tf2_ros


class TransferToGlobal(EventState):
	"""
	Turns coordinates relative to the robot into coordinates relative to total world

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
		self._frame = "puddles/base_link"
		self._start_time = rospy.Time.now()
		self._timeout_temp = 1


	def callback(self,userdata):
		#Changing the coordinate system into the viewpoint of the target to easily move the robot three feet in front of it
		transformed_pose = self.transform_pose(self._pose, self._frame, "world")
		userdata.x = transformed_pose.position.x
		userdata.y = transformed_pose.position.y
		userdata.z = transformed_pose.position.z
		userdata.orientation = transformed_pose.orientation
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