from flexbe_core import EventState, Logger
import rospy
import riptide_controllers.msg
from flexbe_core.proxy import ProxyPublisher 
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import PoseStamped


class SubscriberState(EventState):
    """
	Brings data in from mapping topic. 
    Subscribes to mapping topic.

	-- topic 		string 			Topic to which the pose will be published.

	># pose			PoseStamped		Pose to be published.

	<= done							Pose has been published.

	"""
	def listener():
        ProxySubscriber.init_node('listener', anonymous=True)
		ProxySubscriber("mapping/gate", PoseWithCovarianceStamped, queue_size=1)

    if __name__ == '__main__':
        listener()			## do i need this w proxy?? I used rospy before so I used this to check the name. Does proxy work the same?