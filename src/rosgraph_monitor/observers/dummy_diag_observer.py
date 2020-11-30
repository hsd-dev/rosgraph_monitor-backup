from rosgraph_monitor.observer import TopicObserver
from rosgraph_monitor.output_interfaces.diagnostic_publisher import DiagnosticPublisher
from std_msgs.msg import Int32
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


class DummyDiagnosticObserver(TopicObserver, DiagnosticPublisher):
    def __init__(self, name): #, set_point=None): # reads from param?
        topics = [("/speed", Int32), ("/accel", Int32)]     # list of pairs
        self._set_point = 5.5 #set_point
        DiagnosticPublisher.__init__(self, topic_name="/diagnostics")
        TopicObserver.__init__(self, name=name, loop_rate_hz=10, topics=topics)

    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        attr = msgs[0].data + msgs[1].data
        is_close = False
        if(self._set_point):
            is_close = self.is_close(attr, self._set_point)
        print("{0} + {1}; is_close={2}".format(msgs[0].data, msgs[1].data, is_close))

        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("enery", str(attr)))
        status_msg.message = "QA status"

        return status_msg
