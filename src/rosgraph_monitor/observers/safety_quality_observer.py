from rosgraph_monitor.observer import TopicObserver
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


class SafetyQualityObserver(TopicObserver):
    def __init__(self, name):
        topics = [("/speed", Int32), ("/accel", Int32), ("/Pr", Float32)]     # list of pairs

        super(SafetyQualityObserver, self).__init__(
            name, 10, topics)

    def calculate_attr(self, msgs):
        status_msg = DiagnosticStatus()

        attr = msgs[0].data
        d_break= ((msgs[0].data**2)/(2*msgs[1].data))
      
       # print (d_break)
        normalized_safety=1.0
        if (d_break>msgs[2].data):
            normalized_safety=msgs[2].data/d_break
        print ("{0}".format(d_break))
        print("{0}".format(normalized_safety))
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = self._id
        status_msg.values.append(
            KeyValue("safety_distance", str(normalized_safety)))
        status_msg.message = "QA status"

        return status_msg
