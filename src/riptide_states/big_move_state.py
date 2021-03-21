#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import sys

from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from nav_msgs.msg import Odometry



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
		
		self._pub = ProxyPublisher({"/puddles/linear_velocity": Vector3})
		self.loc_topic = '/puddles/odometry/filtered'
		self.sub = ProxySubscriberCached({self.loc_topic: Odometry})
		moveit_commander.roscpp_initialize(sys.argv)
		#rospy.init_node('test_move_group', anonymous=True)
		
	def execute(self, userdata):
		threshold = .2
		if self.sub.has_msg(self.loc_topic):
			msg = self.sub.get_last_msg(self.loc_topic)
			self.sub.remove_last_msg(self.loc_topic)
			if userdata.x-threshold < msg.pose.pose.position.x < userdata.x +threshold:
				if userdata.y-threshold < msg.pose.pose.position.y < userdata.y+threshold:
					if userdata.z - threshold < msg.pose.pose.position.z < userdata.z+threshold:
						self.move_group.stop()
						del self.move_group
						return 'done'
		
	
	def on_enter(self, userdata):
		self._pub.publish("/puddles/linear_velocity", Vector3(0,0,0))
		rospy.sleep(1)
		self.x = userdata.x
		self.y = userdata.y
		self.z = userdata.z
		self.orientation = userdata.orientation	

		group_name = "puddles_base"
		self.move_group = moveit_commander.MoveGroupCommander(group_name)

		

		

		if self.orientation == None:
			while not self.sub.has_msg(self.loc_topic):
				Logger.loginfo("Waiting for Localization message")
			msg = self.sub.get_last_msg(self.loc_topic)
			self.sub.remove_last_msg(self.loc_topic)
			self.orientation = Quaternion()
			self.orientation.x = msg.pose.pose.orientation.x
			self.orientation.y = msg.pose.pose.orientation.y
			self.orientation.z = msg.pose.pose.orientation.z
			self.orientation.w = msg.pose.pose.orientation.w
			

		joint_goal = [self.x,self.y,self.z,self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]

		self.move_group.set_planning_time(1)
		self.move_group.set_workspace([-50,-50,-30,50,50,0])
		
		
		plan = self.move_group.plan(joint_goal)

		
		self.move_group.execute(plan,wait=False)

		
		