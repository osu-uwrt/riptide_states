#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import sys

from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import PoseStamped

from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class BigMoveState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self):
		"""Constructor"""
		super(BigMoveState, self).__init__(outcomes=['done','failed'],
											input_keys=['x', 'y', 'z', 'orientation'])
		

		self.loc_topic = '/puddles/odometry/filtered'
		self.sub = ProxySubscriberCached({self.loc_topic: Odometry})
		
	def execute(self, userdata):
		return 'done'
	
	def on_enter(self, userdata):
		self.x = userdata.x
		self.y = userdata.y
		self.z = userdata.z
		self.orientation = userdata.orientation


		roscpp_initialize(sys.argv)
		self.robot = RobotCommander()
		msg = Odometry()
		if self.sub.has_msg(self.loc_topic):
			msg = self.sub.get_last_msg(self.loc_topic)
			self.sub.remove_last_msg(self.loc_topic)
		
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		self.z = msg.pose.pose.position.z 
		

		if self.orientation == None:
			self.orientation = Quaternion()
			self.orientation.x = msg.pose.pose.orientation.x
			self.orientation.y = msg.pose.pose.orientation.y
			self.orientation.z = msg.pose.pose.orientation.z
			self.orientation.w = msg.pose.pose.orientation.w
		
		# X, Y, Z, x, y, z, w
		r = [self.x, self.y, self.z, self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]

		self.planner = self.robot.puddles_base
		self.plan = self.planner.plan(r)

		self.planner.execute(self.plan)