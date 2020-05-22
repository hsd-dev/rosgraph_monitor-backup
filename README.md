# rosgraph_monitor

## Installation
```
$ cd <path/to/workspace/src> git clone -b nav_observer https://github.com/ipa-hsd/rosgraph_monitor/
$ cd <path/to/workspace>
$ source /opt/ros/melodic/setup.bash
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin build 
$ source <path/to/workspace/devel/>setup.bash
```
For NavModel.
```
pip3 install roslibpy
https://pypi.python.org/pypi/service_identity
```

## Running the system  
```
roslaunch rosbridge_server rosbridge_websocket.launch
roslaunch rosgraph_monitor demo.launch
python3 src/rosgraph_monitor/nav_model.py
rosservice call /load_observer "name: 'NavObserver'"
```
