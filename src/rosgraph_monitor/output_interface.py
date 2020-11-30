import rospy


class OutputInterface(object):
    def __init__(self):
        pass

    def perform_output():
        pass


class ObservationPublisher(OutputInterface):
    def __init__(self, topic_name, message_type):
        super(ObservationPublisher, self).__init__()

        self._seq = 0;
        self._publisher = rospy.Publisher(
            topic_name, message_type, queue_size=10)

    def perform_output(self, msg):
        try:
            msg.header.stamp = rospy.get_rostime()
        except Exception as exc:
            pass

        self._publisher.publish(msg)
        self._seq += 1

