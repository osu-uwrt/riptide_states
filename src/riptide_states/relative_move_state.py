#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3

import tf

class RelativeMoveState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, x, y, z):
		"""Constructor"""
		super(RelativeMoveState, self).__init__(outcomes=['Success', 'Failure'])
		self._x = x
		self._y = y
		self._z = z
		self._frame = "/puddles/base_link"
		self.tl = tf.TransformListener()
		self.loc_topic = '/puddles/odometry/filtered'
		self.sub = ProxySubscriberCached({self.loc_topic: Odometry})
		self._topic = "puddles/position"
		self._pub = ProxyPublisher({self._topic: Vector3})
		


	def execute(self, userdata):
		threshold = .2
		if self.sub.has_msg(self.loc_topic):
			msg = self.sub.get_last_msg(self.loc_topic)
			self.sub.remove_last_msg(self.loc_topic)
			if userdata.x-threshold < msg.pose.pose.position.x < userdata.x +threshold:
				if userdata.y-threshold < msg.pose.pose.position.y < userdata.y+threshold:
					if userdata.z - threshold < msg.pose.pose.position.z < userdata.z+threshold:
						return 'Success'
		
	def on_enter(self, userdata):
		#find the most recent transform
		t = self.tl.getLatestCommonTime(self._frame,"/world")
		#populate our query
		pl = PoseStamped()
		pl.header.frame_id = self._frame
		pl.header.stamp = t
		
		pl.pose.position.x = self._x
		pl.pose.position.y = self._y
		pl.pose.position.z = self._z
		

		convertedPos = PoseStamped()
		convertedPos = self.tl.transformPose("/world", pl)

		msg = Vector3()

		msg.x = convertedPos.pose.position.x
		msg.y = convertedPos.pose.position.y
		msg.z = convertedPos.pose.position.z

		self._pub.publish(self._topic,msg)


