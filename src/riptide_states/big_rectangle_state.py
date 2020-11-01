#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import darknet_ros_msgs.msg._BoundingBoxes as BoundingBoxes
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher
from geometry_msgs.msg import PoseStamped


class BigRectangleState(EventState):
	"""
	Publishes the rectangular coordinates of a specified target to an inputed topic.

	-- topic 		string 			Topic to which the pose will be published.

	># target	    string	        What is trying to be found (Gate, Batman, Fairy, ec)

	<= Success						Coordinates are published.

    <= Failed                       Coordinates are not published.

    <= Lost_Visual                  Coordinates are not published.

	"""
	
	def __init__(self, topic, threshold, time, target, timeout):
		"""Constructor"""
		super(BigRectangleState, self).__init__(outcomes=['Success', 'Failed'], output_keys=['x','y'])
		self._threshold = threshold
		self._target_time = time
		self._start_time = rospy.Time.now()
		self._timeout = timeout
		self._topic = topic
		self._target = target
		#self._pub = ProxyPublisher({self._topic: PoseStamped})
		self._sub = ProxySubscriberCached({self._topic: BoundingBoxes})
		self._potential = []




	def callback(self,bbox,userdata):
		self.maxProbability = self._threshold
		for box in bbox.bounding_boxes:
			if box.Class == self._target and box.probability >= self.maxProbability:
				#Check if any previous bboxes are the same
				found = False
				for seenBox in self._potential:
					if box.ymin <= seenBox.ymin +10 | box.ymin >= seenBox.ymin -10:
						found = True
						if bbox.bounding_boxes.header.stamp.nseq - seenBox.time >=self._target_time:
							userdata.x = (box.xmax+box.xmin)/2
							userdata.y = (box.ymax+box.ymin)/2
							return 'Success'
				#If the new box isn't on the potential list, add it.
				if not found:
					self._potential[len(self._potential)+1].box = box
					self._potential[len(self._potential)+1].time = bbox.header.stamp.nseq

		return 'Failed'


	def execute(self,userdata):
		if self._sub.has_msg(self._topic):
			msg = self._sub.get_last_msg(self._topic)
			self._sub.remove_last_msg(self._topic)
		if self.callback(msg,userdata) == 'Success':
			return 'Success'
		if self._start_time - rospy.Time.now() > self._timeout:
			return 'Failed'
