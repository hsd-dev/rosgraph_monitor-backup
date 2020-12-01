from rosgraph_monitor.observer import TopicObserver
from rosgraph_monitor.output_interface import ObservationOutputPublisher
from std_msgs.msg import Int32, Bool
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


# example observer to publish result as a DiagnosticStatus message
class DummyDiagnosticObserver(TopicObserver, ObservationOutputPublisher):
    def __init__(self, name): #, set_point=None): # reads from param?
        topics = [("/speed", Int32), ("/accel", Int32)]     # list of pairs
        self._set_point = 5.5 #set_point
        ObservationPublisher.__init__(self, topic_name="/diagnostics", DiagnosticStatus)
        TopicObserver.__init__(self, name=name, loop_rate_hz=10, topics=topics)

    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        attr = msgs[0].data + msgs[1].data
        is_close = True
        if(self._set_point):
            is_close = self.is_close(attr, self._set_point, 0.05)
        print("{0} + {1}; is_close={2}".format(msgs[0].data, msgs[1].data, is_close))

        status_msg.level = DiagnosticStatus.OK if is_close else DiagnosticStatus.ERROR
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("enery", str(attr)))
        status_msg.message = "QA status"

        return status_msg


# example observer to publish result as a Bool message
class DummyBoolObserver(TopicObserver, ObservationPublisher):
    def __init__(self, name): #, set_point=None): # reads from param?
        topics = [("/speed", Int32), ("/accel", Int32)]     # list of pairs
        self._set_point = 5.5 #set_point
        ObservationPublisher.__init__(self, topic_name="/output_topic", Bool)
        TopicObserver.__init__(self, name=name, loop_rate_hz=10, topics=topics)

    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        attr = msgs[0].data + msgs[1].data
        is_close = False
        if(self._set_point):
            is_close = self.is_close(attr, self._set_point, 0.05)
        print("{0} + {1}; is_close={2}".format(msgs[0].data, msgs[1].data, is_close))

        msg = Bool()
        msg.data = is_close
        return msg
