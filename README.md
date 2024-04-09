# Robotic_HW - Gabriel Bronfman
## Overview
This workspace contains two packages, sensor_srv and robot_interfaces, that work together to provide a service that exposes close to real-time sensor data from a generic TCP/IP connected load cell(s). The load cell in this case has 6 DOF. At runtime, the service server node generically accepts a specified number of sensors, and connects to them according to a defined list of addresses. At this time each one operates on the same port number, though with relatively simple modification that can be specified at runtime in a future version as well.

Each sensor is given its own thread to handle concurrency issues to make data available to service requesters with as little latency as possible. Given that a load cell at the end of the robot is a control sensitive piece of information, special attention was given to providing as close to real-time data as possible. The overhead from service request to data serve is the sum of all processing and measuring. It takes $1ms$ of network latency per request, plus $\frac{n}{2000}$ measuring time to get $n$ samples. My system has an overhead from service request to response of between $.2ms$ to $.4ms$. The specifications require publication of requested information at $500hz$. I chose $5$ samples per request to create a continuous flow of data at 100 service requests per second. I thought that struck a good balance between, in the worst case, $4ms$ of latency and not overloading a busy communication network while providing a continuous stream of data with no interruptions.
 
## Deploying
Any computer with a connected internet connection can simply clone the repository, extract desired packages, and place them in the the ```src``` directory of a predefined ```colcon_ws```. Simply run ```colcon build``` in the root of the workspace and source your workspace.

Ensure that your sensor is connected to the computer via ethernet or is on the same network, with a defined IP that you can ping. If required, enable communication to occurr to the IP if your firewall prohibits unknown connections. 

## Runtime
The service server is a lifecycle node, meaning that its state can be configured programatically, or directly via the ROS2 CLI. Below is an image of the relevant states, and what important characteristics are present at each state:

![Screenshot of state diagram](/ros2_lifecycle_node.png)

Importantly, the node can deactivate and maintain its connection with the TCP/IP connected device, but will not expose service functionality to the rest of the ROS2 graph. Additionally, you must manually activate the node to expose service request functionality.

## Parameters
The sensor_server_node has a couple of parameters that can be specified via the sensor_params.yaml file. These are:
* num_sensor: The number of desired load-cells you want to connect to. Threads will be directly allocated to handle each connection to ensure performance. Spawning too many threads could be very costly.
* sensor_addresses: a list of index matched sensor addresses that specify the IP of each sensor defined by num_sensors. Node WILL NOT configure if any of the addresses are incorrect or if node fails to connect to any sensors. This ensures that we are never in a situation where the application thinks that everything is configured properly, but some sensors are not operating nominally
* port: This number will be the port for all sensors when their respective sockets are initialized. Can be updated at a later date to accomodate multiple ports

## Demo

To demo this repository, ensure you have built the workspace, and sourced the workspace (in each terminal). From the root of the repo you can run the following commands in seperate terminals:

To start the sensor_server_node with parameters that work for my sensor.py, you can run:

```
ros2 run sensor_srv sensor_server_node --ros-args --params-file machina_ws/src/sensor_srv/config/sensor_params.yaml 
```

To start the sensor.py, you can exectute:

```
python3 sensor.py
```

The simple client for publication has to be run twice, where each one has a specified parameter "sensor_num" so that they publish to seperate topics. These are the commands:

```
ros2 run sensor_srv sensor_info_publisher --ros-args -r sensor_info_service:=sensor_info_0 -p sensor_number:=0
```
```
ros2 run sensor_srv sensor_info_publisher --ros-args -r sensor_info_service:=sensor_info_1 -p sensor_number:=1
```

Now you can configure and activate the lifecycle node using:

```
ros2 lifecycle set /sensor_server_node configure
ros2 lifecycle set /sensor_server_node activate
```

You should be able to ```ros2 topic list``` and see two topics, one for each sensor, that publishes received data in real-time.
