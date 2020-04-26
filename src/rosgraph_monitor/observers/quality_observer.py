from rosgraph_monitor.observer import TopicObserver
from std_msgs.msg import Int32


class QualityObserver(TopicObserver):
    def __init__(self, name):
        param = "/quality"
        topics = [("/speed", Int32), ("/accel", Int32)]     # list of pairs

        super(QualityObserver, self).__init__(
            name, 10, param, topics)

    def perform_check(self, msgs):
        if len(msgs) < 2:
            return False, "Incorrect number of messages"
        if not isinstance(msgs[0], Int32) or not isinstance(msgs[1], Int32):
            return False, "Incorrect instance of message"

        isLower = (msgs[0].data + msgs[1].data) < int(self._param_val)
        print(
            "{0} + {1} < {2}".format(msgs[0].data, msgs[1].data, self._param_val))

        return isLower, "Higher than expected"  # Error message
