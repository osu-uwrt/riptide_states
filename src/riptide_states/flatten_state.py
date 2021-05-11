#!/usr/bin/env python3

from flexbe_core import EventState, Logger
import rospy

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from std_msgs.msg import Bool

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import tf

class FlattenState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self):
		"""Constructor"""
		super(FlattenState, self).__init__(outcomes=['Success', 'Failure'])
		
		self._frame = "/puddles/base_link"
		self.steady_topic = '/puddles/steady'
		self.tl = tf.TransformListener()
		self.loc_topic = '/puddles/odometry/filtered'
		self.sub = ProxySubscriberCached({self.loc_topic: Odometry, self.steady_topic: Bool})
		self._topic = "puddles/orientation"
		self._postopic = "puddles/position"
		self._pub = ProxyPublisher({self._topic: Quaternion, self._postopic: Vector3})
		


	def execute(self, userdata):
		Logger.loginfo("Waiting for Orientation to complete")
		
		if self.sub.has_msg(self.steady_topic):
			msg = self.sub.get_last_msg(self.steady_topic)
			self.sub.remove_last_msg(self.steady_topic)
			if msg.data:
					Logger.loginfo("Done Orientating")
					return 'Success'
		
	def on_enter(self, userdata):
		#find the most recent transform
		t = self.tl.getLatestCommonTime(self._frame,"world")
		#populate our query
		

		pl = PoseStamped()
		pl.header.frame_id = self._frame
		pl.header.stamp = t
		
		pl.pose.position.x = 0
		pl.pose.position.y = 0
		pl.pose.position.z = -.5
	
		convertedPos = self.tl.transformPose("world", pl)

		msg = Vector3()
		
		msg.x = convertedPos.pose.position.x
		self._x=msg.x
		msg.y = convertedPos.pose.position.y
		self._y=msg.y
		msg.z = convertedPos.pose.position.z
		self._z=msg.z
		Logger.loginfo("Moving: {}, {}, {}".format(msg.x,msg.y,msg.z))
		self._pub.publish(self._postopic,msg)


		while not self.sub.has_msg(self.loc_topic):
			#do nothing
			x = None

		msg = self.sub.get_last_msg(self.loc_topic)

		
		
		initialOri = msg.pose.pose.orientation

		initialQuat = [initialOri.x,initialOri.y,initialOri.z,initialOri.w]

		euler = euler_from_quaternion(initialQuat)		

		initialQuat = quaternion_from_euler(0,0,euler[2])

		reqOri = Quaternion()

		reqOri.x = initialQuat[0]
		reqOri.y = initialQuat[1]
		reqOri.z = initialQuat[2]
		reqOri.w = initialQuat[3]
		
		Logger.loginfo("Requesting Orientation: {}".format(reqOri))
		self._pub.publish(self._topic,reqOri)


