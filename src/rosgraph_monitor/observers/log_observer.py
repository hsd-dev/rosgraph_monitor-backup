from rosgraph_monitor.observer import Observer
from diagnostic_msgs.msg import DiagnosticArray


class LogObserver(Observer):
    def __init__(self, name):
        super(LogObserver, self).__init__(name, 1)

    def generate_diagnostics(self):
        msg = DiagnosticArray()
        return msg
