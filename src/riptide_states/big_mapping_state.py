from flexbe_core import EventState, Logger
import rospy
import riptide_controllers.msg
from flexbe_core.proxy import ProxyPublisher 
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import PoseStamped


class BigMappingState(EventState):
    """
	Brings data in from mapping topic. 
    Subscribes to mapping topic.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, topic):
		"""Constructor"""
		super(BigMappingState, self).__init__(outcomes=['Success', 'Failure'],
            input_keys=['angle'])
		self._topic = topic
		self.sub = ProxySubscriber("/puddles/mapping/gate", PoseWithCovarianceStamped, queue_size=1)

	def execute(self, userdata):
		if self.client.has_result(self._topic):
			result = self.client.get_result(self._topic)
			status = 'Success'       
			return status



	