#!/usr/bin/env python

import importlib
import time
import inspect
import pkgutil

import rospy
import rosgraph_monitor.observers
from controller_manager_msgs.srv import *


def iter_namespace(ns_pkg):
    return pkgutil.iter_modules(ns_pkg.__path__, ns_pkg.__name__ + ".")


class ModuleManager(object):
    def __init__(self):
        self._modules = {}
        self._observers = {}
        rospy.Service('/load_observer', LoadController, self.handle_load)
        rospy.Service('/unload_observer', UnloadController, self.handle_unload)
        rospy.Service('/active_observers',
                      ListControllerTypes, self.handle_active)
        rospy.Service('/list_observers',
                      ListControllerTypes, self.handle_types)
        
    def handle_load(self, req):
        started = self.start_observer(req.name)
        return LoadControllerResponse(started)

    def handle_unload(self, req):
        stopped = self.stop_observer(req.name)
        return UnloadControllerResponse(stopped)

    def handle_active(self, req):
        names = self._observers.keys()
        return ListControllerTypesResponse(names, [])

    def handle_types(self, req):
        names = self._modules.keys()
        return ListControllerTypesResponse(names, [])

    def load_observers(self):
        available_plugins = {
            name: importlib.import_module(name)
            for finder, name, ispkg
            in iter_namespace(rosgraph_monitor.observers)
        }
        self._modules = self._get_leaf_nodes(
            rosgraph_monitor.observer.Observer)
        
    def start_observer(self, name):
        started = True
        try:
            module = self._modules[name]
            self._observers[name] = getattr(module, name)(name)
            self._observers[name].start()
        except Exception as exc:
            print("Could not start {}".format(name))
            started = False
        return started

    def stop_observer(self, name):
        stopped = True
        try:
            self._observers[name].stop()
            del self._observers[name]
        except Exception as exc:
            print("Could not stop {}".format(name))
            stopped = False
        return stopped

    def _get_leaf_nodes(self, root):
        leafs = {}
        self._collect_leaf_nodes(root, leafs)
        return leafs

    def _collect_leaf_nodes(self, node, leafs):
        if node is not None:    # change this to see if it is class
            if len(node.__subclasses__()) == 0:
                leafs[node.__name__] = inspect.getmodule(node)
            for n in node.__subclasses__():
                self._collect_leaf_nodes(n, leafs)


if __name__ == "__main__":
    rospy.init_node('graph_monitor')

    manager = ModuleManager()
    manager.load_observers()

    rospy.spin()
