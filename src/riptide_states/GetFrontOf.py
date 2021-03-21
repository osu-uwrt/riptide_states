#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
import tf
from enum import Enum


gatePos = Pose()
gatePos.position.x = 1
gatePos.position.y = -3
gatePos.position.z = 0
gatePos.orientation.x = 0
gatePos.orientation.y = 0
gatePos.orientation.z =  0.707
gatePos.orientation.w =  0.707

polePos = Pose()
polePos.position.x = -3
polePos.position.y = 0
polePos.position.z = 0
polePos.orientation.x = 0
polePos.orientation.y = 0
polePos.orientation.z =  0
polePos.orientation.w =  1

class GetFrontOf(EventState):
	"""
	Changes the input position to be 3 feet in front of the object.

	<= Success						Coordinates are published.

    <= Failed                       Coordinates are not published.

	"""

	def transform_pose(self, input_pose, from_frame, to_frame):
		
		t = self.tl.getLatestCommonTime(from_frame, to_frame)
		p1 = PoseStamped()
		p1.header.frame_id = from_frame
		p1.header.stamp = t
		p1.pose = input_pose
		convertedPos = self.tl.transformPose(to_frame, p1)
		return convertedPos


	def __init__(self,target):
		"""Constructor"""
		super(GetFrontOf, self).__init__(outcomes=['Success'], output_keys=['x','y','z','orientation'])
		self._frame = target
		if(self._frame=="pole_frame"):
			self._transformed_pose = polePos
		elif(self._frame=="gate_frame"):
			self._transformed_pose = gatePos
		#Logger.loginfo(target)
		self._start_time = rospy.Time.now()
		self._timeout_temp = 1
		self.tl = tf.TransformListener()


	def callback(self,userdata):
		#Changing the coordinate system into the viewpoint of the target to easily move the robot three feet in front of it
		transformed_pose = self._transformed_pose
		#Changing the coordinates back into global
		_updated_pose = self.transform_pose(transformed_pose,self._frame,"/world")
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