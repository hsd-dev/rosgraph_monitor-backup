from rosgraph_monitor.observer import TopicObserver
import rospy
from std_msgs.msg import Float32MultiArray
from rosgraph_monitor.srv import PredictAction, PredictActionRequest
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


class NavObserver(TopicObserver):
    def __init__(self, name):
        topics = [("/radar", Float32MultiArray)]

        super(NavObserver, self).__init__(
            name, 10, topics)

        rospy.wait_for_service('/turtle_action', timeout=1.0)
        self.client = rospy.ServiceProxy('/turtle_action', PredictAction)

    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        req = PredictActionRequest()
        req.sensor_data = msgs[0].data
        resp = self.client(req)
        attr = resp.action
        # print("Predicted action: {0}".format(attr))

        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("action", str(attr)))
        status_msg.message = "Action prediction"

        print(status_msg)

        return status_msg
