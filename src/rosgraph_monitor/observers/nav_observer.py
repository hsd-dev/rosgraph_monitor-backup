from rosgraph_monitor.observer import TopicObserver
from rosgraph_monitor.nav_model import NavModel
from std_msgs.msg import Float32MultiArray
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


class NavObserver(TopicObserver):
    def __init__(self, name):
        topics = [("/radar", Float32MultiArray)]     # list of pairs

        super(NavObserver, self).__init__(
            name, 10, topics)

        self._model = NavModel(
            'src/rosgraph_monitor/resources/sensor_readings_24.csv', 24)
        self._model.prepare_data()
        self._model.train()

    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        sensor_data = msgs[0].data
        attr = self._model.get_action(sensor_data)
        print("Predicted action: {0}".format(attr))

        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("action", str(attr)))
        status_msg.message = "Action prediction"

        print(status_msg)

        return status_msg
