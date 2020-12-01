import rospy


class OutputInterface(object):
    def __init__(self):
        pass

    def perform_output():
        pass


class ObservationOutputPublisher(OutputInterface):
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


class ObservationOutputLogger(OutputInterface):
    def __init__(self, file_name):
        pass

    def perform_output(self, msg):
        # do logging
        pass


class ObservationOutputPlotter(OutputInterface):
    def __init__(self, plt_params):
        pass

    def perform_output(self, msg):
        # do plotting
        pass
