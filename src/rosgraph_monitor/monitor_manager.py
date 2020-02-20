#!/usr/bin/env python

import threading
import mutex
import rospy
from diagnostic_msgs.msg import DiagnosticArray


class ServiceWrapper(object):
    def __init__(self, service_name=None, service_type=None):
        self.name = service_name
        self.type = service_type
        self.client = None

    def generate_diagnostics(self):
        resp = self.client.call()  # do I need a try catch here?
        status_msg = self.diagnostics_from_response(resp)
        return status_msg

    # Every derived class needs to override this
    def diagnostics_from_response(self, response):
        msg = DiagnosticArray()
        return msg


class MonitorManager(object):
    def __init__(self):
        loop_rate_hz = 1
        rate = rospy.Rate(loop_rate_hz)

        self._pub_diag = rospy.Publisher(
            'diagnostics', DiagnosticArray, queue_size=10)
        self._services = []
        self._ser_lock = threading.Lock()
        self._thread = threading.Thread(
            target=self.call_all, args=(rate,))
        self._thread.daemon = True

    # wrong service not caught properly
    # ERROR (in case of wrong type): thread.error: release unlocked lock
    def register_service(self, service):
        try:
            rospy.wait_for_service(service.name, timeout=1.0)
            service.client = rospy.ServiceProxy(service.name, service.type)
            self._ser_lock.acquire()
            self._services.append(service)
            print("Service '" + service.name +
                  "' added of type" + str(service.type))
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        finally:
            self._ser_lock.release()

    def call_all(self, rate):
        seq = 1
        while not rospy.is_shutdown():
            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = rospy.get_rostime()

            self._ser_lock.acquire()
            for service in self._services:
                status_msg = service.generate_diagnostics()
                diag_msg.status.append(status_msg)

            self._pub_diag.publish(diag_msg)
            self._ser_lock.release()
            seq += 1
            rate.sleep()

    def loop(self):
        self._thread.start()
