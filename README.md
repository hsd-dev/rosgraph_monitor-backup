# rosgraph_monitor
Monitors the status of nodes and topics currently running.

Triggers notifications on changes in the ROS graph(?)
  - new node connected, node not responding, etc
  - based on the monitor "requested for"
    - topic publish hz is "too low"

**Reference repos:**  
Utilize Log messages to read the node status?  
http://docs.ros.org/api/rosgraph_msgs/html/msg/Log.html

Implementation of EMF
https://github.com/pyecore/pyecore

http://docs.ros.org/melodic/api/diagnostic_msgs/html/msg/DiagnosticStatus.html  

**Terminologies:** (from OPC-UA)  
MonitoredItem - entities to monitor *Nodes*, create *Notification* when change detected for clients through *Subscriptions*  
Subscription - publishes *Notifications* to client
