import threading
import rospy


class Observer(object):
    def __init__(self, name, loop_rate_hz=1):
        self._name = name
        self._rate = rospy.Rate(loop_rate_hz)
        self._lock = threading.Lock()
        self._thread = threading.Thread(
            target=self._run)
        self._thread.daemon = True
        self._stop_event = threading.Event()

    def __del__(self):
        if Observer:
            print("{} stopped".format(self._name))

    # Every derived class needs to override this
    # def generate_diagnostics(self):
    #     msg = DiagnosticArray()
    #     return msg
    def generate_output():
        return None

    def _run(self):
        while not rospy.is_shutdown() and not self._stopped():
            msg = self.generate_output()
            print(msg)

            # perform_output() should be implemented by OutputInterface's sub-class
            self._perform_output(msg)

            self._rate.sleep()

    def start(self, func):
        print("starting {}...".format(self._name))
        self._perform_output = func
        self._thread.start()

    def stop(self):
        self._lock.acquire()
        self._stop_event.set()
        self._lock.release()

    def _stopped(self):
        self._lock.acquire()
        isSet = self._stop_event.isSet()
        self._lock.release()
        return isSet


class ServiceObserver(Observer):
    def __init__(self, name, service_name=None, service_type=None, loop_rate_hz=1):
        super(ServiceObserver, self).__init__(name, loop_rate_hz)
        self.name = service_name
        self.type = service_type
        self.client = None
        try:
            self.start_service()
        except:
            print("{} service not started".format(self.name))
            super.__del__()

    def start_service(self):
        try:
            rospy.wait_for_service(self.name, timeout=1.0)
            self.client = rospy.ServiceProxy(self.name, self.type)
            print("Service '" + self.name +
                  "' added of type" + str(self.type))
        except rospy.ServiceException as exc:
            print("Service {} is not running: ".format(self.name) + str(exc))

    def generate_output(self):
        try:
            resp = self.client.call()
        except rospy.ServiceException as exc:
            print("Service {} did not process request: ".format(
                self.name) + str(exc))
        # status_msg = self.diagnostics_from_response(resp)
        # return status_msg

    # Every derived class needs to override this
    # def diagnostics_from_response(self, response):
    #     msg = DiagnosticArray()
    #     return msg


class TopicObserver(Observer):
    def __init__(self, name, loop_rate_hz, topics):
        super(TopicObserver, self).__init__(name=name, loop_rate_hz=loop_rate_hz)
        self._topics = topics
        self._id = ""
        self._num_topics = len(topics)

    def is_close(self, a, b, rel_tol=1e-09, abs_tol=0.0):
        return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

    # Every derived class needs to override this
    def calculate_attr(self, msgs):
        # do calculations
        return None

    def generate_output(self):
        msgs = []
        received_all = True
        for topic, topic_type in self._topics:
            try:
                # Add message synchronization here -- message filter
                # Consider "asynchronous" reception as well?
                msgs.append(rospy.wait_for_message(topic, topic_type))
            except rospy.ROSException as exc:
                print("Topic {} is not found: ".format(topic) + str(exc))
                received_all = False
                break

        msg = None
        if received_all:
            msg = self.calculate_attr(msgs)

        return msg
