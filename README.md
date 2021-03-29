# rosgraph_monitor

## Installation
```
$ cd <path/to/workspace/src> git clone -b observers https://github.com/ipa-hsd/rosgraph_monitor/
$ cd <path/to/workspace/src> git clone -b SoSymPaper https://github.com/ipa-nhg/ros_graph_parser
$ cd <path/to/workspace>
$ source /opt/ros/melodic/setup.bash
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin build 
$ source <path/to/workspace/devel/>setup.bash
```

## Running the system  
source the workspace in all the terminals

```
# Terminal 1
$ roscore

# Terminal 2
$ rosrun rosgraph_monitor monitor

# Publish the topics listed in the `SafetyQualityObserver`  and 'EnergyQualityObserver'

# In a new terminal 
$ rosservice call /load_observer "name: 'SafetyQualityObserver'"

# In a new terminal 
$ rosservice call /load_observer "name: 'EnergyQualityObserver'"
```
