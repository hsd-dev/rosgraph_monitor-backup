from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rosgraph_monitor.output_interface import ObservationPublisher

class DiagnosticPublisher(ObservationPublisher):
    def __init__(self, topic_name):
        super(DiagnosticPublisher, self).__init__(topic_name=topic_name, message_type=DiagnosticStatus)
