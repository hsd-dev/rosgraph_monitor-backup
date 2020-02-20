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


class ModelParser(object):
    def __init__(self, model):
        self.nodes = list()
        # connections defined specifically in the rossystem file
        self.topic_connections = dict()
        self.service_connections = dict()
        self.action_connections = dict()
        # expected connections if matched names of interfaces
        self.auto_topic_connections = dict()
        self.auto_service_connections = dict()
        self.auto_action_connections = dict()
        
        self.model = model

    ## GET A LIST OF STRINGS WITH THE NAMES OF ALL COMPONENTS DEFINED ON THE ROSSYSTEM MODEL
    def get_nodes(self):
        for i in range(1,self.model.count("ComponentInterface")+1):
            self.nodes.append(self.model.split('ComponentInterface')[i].split("{ name")[1].split()[0].replace("'","").replace('"',''))

    ## ANALYZE THE INPUT MODEL AND CHECK THE PREDEFINED CONNECTIONS
    ## GET A DICT OF CONNECTIONS WITH THE FORMAT -- {CONNECTION_NAME : ( FROM_NODE_NAME , TO_NODE_NAME )}
    def get_connections_from_model(self):
        self.topic_connections =self.get_connections_helper(self.model,'TopicConnection',1) 
        self.service_connections =self.get_connections_helper(self.model,'ServiceConnection',1)
        self.action_connections =self.get_connections_helper(self.model,'ActionConnection',1)

    ## ANALYZE THE INPUT MODEL, LIST ALL THE INTERFACES, COMPARE THEM AND EXTRACT THE CONNECTIONS (I.E. SAME PATTERN AND SAME)
    ## GET A DICT OF CONNECTIONS WITH THE FORMAT -- {CONNECTION_NAME : ( FROM_NODE_NAME , TO_NODE_NAME )}
    def compute_connections(self):
        components=self.split_list(self.model,"ComponentInterface","TopicConnections",0)
        components_dict=dict()

        if components:
            for c in components:
                if "name" in c:
                    component_name=c.split("{ name")[1].split()[0].replace("'","").replace('"','')
                    components_dict[component_name] = [self.get_interfaces_name(c,"Publisher", "Publisher",1),
                                                       self.get_interfaces_name(c,"Subscriber","Subscriber",1),
                                                       self.get_interfaces_name(c,"ServiceServer","Server",0),
                                                       self.get_interfaces_name(c,"ServiceClient","Client",0),
                                                       self.get_interfaces_name(c,"ActionServer","Server",1),
                                                       self.get_interfaces_name(c,"ActionClient","Client",1)]
        if components_dict:
            for i in components_dict:
                for j in components_dict:
                    if i!=j:
                        if components_dict[i][0] and components_dict[j][1]:
                            for pub in components_dict[i][0]:
                                for sub in components_dict[j][1]:
                                    if pub==sub:
                                        self.auto_topic_connections[pub]=(i,j)
                        if components_dict[i][2] and components_dict[j][3]:
                            for srv in components_dict[i][2]:
                                for cli in components_dict[j][3]:
                                    if srv==cli:
                                        self.auto_services_connections[srv]=(i,j)
                        if components_dict[i][4] and components_dict[j][5]:
                            for srv in components_dict[i][4]:
                                for cli in components_dict[j][5]:
                                    if srv==cli:
                                        self.auto_actions_connections[srv]=(i,j)

    ## HELPERS ##
    def get_connections_helper(self, model, keyword, offset):
        connections =dict()
        interface_connections=self.split_list(model,keyword,'} }',offset)

        if interface_connections:
            for i in interface_connections:
                name_connection=i.split()[0].replace("'","").replace('"','')
                from_connection=i.split('From (')[1].split()[0].replace("'","").replace('"','')
                to_connection=i.split('To (')[1].split()[0].replace("'","").replace('"','')
                connections[name_connection] = (from_connection,to_connection)
            return connections

    def split_list(self,input_str,keyword,end,offset):
        return_list=list()
        if keyword in input_str:
            split_str=input_str.split(keyword)
            for i in range(1,len(split_str)):
                my_string = split_str[i]
                if (end!=None and my_string.find(end) != -1):
                    return_list.append(my_string[offset:my_string.find(end)])
                else:
                    return_list.append(my_string)
            return return_list[offset:]

    def get_interfaces_name(self,component,keyword,ref_keyword,offset):
        interfaces_name=list()

        interfaces_list=self.split_list(component,"Ros"+keyword,"Ref"+keyword,0)
        if interfaces_list:
            for i in interfaces_list[offset:]:
                interfaces_name.append(i.split()[0].replace("'","").replace('"',''))            
            return interfaces_name


