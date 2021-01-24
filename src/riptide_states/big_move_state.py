#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import sys
from flexbe_core.proxy import ProxyActionClient
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import PoseStamped
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from nav_msgs.msg import Odometry



class BigMoveState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, topic, group, x, y, z, threshold, timeout):
		super(BigMoveState, self).__init__(outcomes=['Success', 'Failed'])
		self._topic = topic
		moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander()
		self._scene =  moveit_commander.PlanningSceneInterface()
		self._group_name = group
		self._odom = "/puddles/odometry/filtered"
		self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
		#self.client = ProxyActionClient({self._topic: PoseStamped})
		self._sub = ProxySubscriberCached({self._odom: Odometry})
		self._x = x
		self._y = y
		self._z = z
		self._threshold = threshold
		self._timeout = rospy.Duration.from_sec(timeout)
		



	def execute(self, userdata):
		if self._sub.has_msg(self._odom):
			msg = self._sub.get_last_msg(self._odom)
			self._sub.remove_last_msg(self._odom)
			if self._x + self._threshold > msg.pose.pose.position.x and self._x - self._threshold < msg.pose.pose.position.x:
				if self._y + self._threshold > msg.pose.pose.position.y and self._y - self._threshold < msg.pose.pose.position.y:
					if self._z + self._threshold > msg.pose.pose.position.z and self._z - self._threshold < msg.pose.pose.position.z:
						status = 'Success'     #hi  
						return status
		if rospy.Time.now() > self._start_time + self._timeout:
			status = 'Failed'
			return status
	
	def on_enter(self, userdata):
		self._start_time = rospy.Time.now()
		msg = self._sub.get_last_msg(self._odom)		
		if(self._x is None):
			self._x = msg.pose.pose.position.x
		if(self._y is None):
			self._y = msg.pose.pose.position.y
		if(self._z is None):
			self._z = msg.pose.pose.position.z
		
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 1.0
		pose_goal.position.x = self._x
		pose_goal.position.y = self._y
		pose_goal.position.z = self._z
		self._move_group.set_pose_target(pose_goal)
		plan = self._move_group.go(wait=True)
		self._move_group.stop()
		self._move_group.clear_pose_targets()

