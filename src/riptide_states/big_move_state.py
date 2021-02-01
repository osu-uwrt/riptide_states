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


class BigMoveState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, x,y,z,orientation):
		"""Constructor"""
		super(BigMoveState, self).__init__(outcomes=['done','failed'])
		self.x = x
		self.y = y
		self.z = z
		self.orientation = orientation

		self.loc_topic = '/Puddles/odometry/filtered'
		self.sub = ProxySubscriberCached({self.loc_topic: Odometry})
		

		
		


	def execute(self, userdata):
		return 'done'
	
	def on_enter(self, userdata):
		roscpp_initialize(sys.argv)
		self.robot = RobotCommander()
		msg = Odometry()
		if self.sub.has_msg(self.loc_topic):
			msg = self.sub.get_last_msg()
			self.sub.remove_last_msg(self.loc_topic)
		
		self.x += msg.pose.pose.x
		self.y += msg.pose.pose.y
		self.z += msg.pose.pose.z 

		# X, Y, Z, x, y, z, w
		r = [self.x, self.y, self.z, 0, 0, 0, 1]

		self.planner = self.robot.puddles_base
		self.plan = self.planner.plan(r)

		self.planner.execute(self.plan)