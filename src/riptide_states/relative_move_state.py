#!/usr/bin/env python3

from flexbe_core import EventState, Logger
import rospy

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool

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
		self.steady_topic = '/puddles/steady'
		self.tl = tf.TransformListener()
		self.loc_topic = '/puddles/odometry/filtered'
		self.sub = ProxySubscriberCached({self.loc_topic: Odometry, self.steady_topic: Bool})
		self._topic = "puddles/position"
		self._pub = ProxyPublisher({self._topic: Vector3})
		


	def execute(self, userdata):
		Logger.loginfo("Waiting for move to complete")
		
		if self.sub.has_msg(self.steady_topic):
			msg = self.sub.get_last_msg(self.steady_topic)
			self.sub.remove_last_msg(self.steady_topic)
			if msg.data:
					Logger.loginfo("Done Moving")
					return 'Success'
		
	def on_enter(self, userdata):
		#find the most recent transform
		t = self.tl.getLatestCommonTime(self._frame,"world")
		#populate our query
		pl = PoseStamped()
		pl.header.frame_id = self._frame
		pl.header.stamp = t
		
		pl.pose.position.x = self._x
		pl.pose.position.y = self._y
		pl.pose.position.z = self._z
	
		convertedPos = self.tl.transformPose("world", pl)

		msg = Vector3()
		
		msg.x = convertedPos.pose.position.x
		self._x=msg.x
		msg.y = convertedPos.pose.position.y
		self._y=msg.y
		msg.z = convertedPos.pose.position.z
		self._z=msg.z
		Logger.loginfo("Moving: {}, {}, {}".format(msg.x,msg.y,msg.z))
		self._pub.publish(self._topic,msg)

		rospy.sleep(3)


