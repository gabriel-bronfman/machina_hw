# Robotic_HW - Gabriel Bronfman
## Overview
This workspace contains two packages, sensor_srv and robot_interfaces, that work together to provide a service that exposes close to real-time sensor data from a generic TCP/IP connected load cell(s). The load cell in this case has 6 DOF. At runtime, the service server node generically accepts a specified number of sensors, and connects to them according to a defined list of addresses. At this time each one operates on the same port number, though with relatively simple modification that can be specified at runtime in a future version as well.

Each sensor is given its own thread to handle concurrency issues to make data available to service requesters with as little overhead as possible. Given that a load cell at the end of the robot is a control sensitive piece of information, special attention was given to providing as close to real-time data as possible. The overhead from service request to data serve is the sum of all processing and measuring. It takes $1ms$ of network latency per request, plus $\frac{n}{2000}$ measuring time to get $n$ samples. My system has an overhead from service request to response of between $.2ms$ to $.4ms$. The specifications require publication of requested information at $500hz$. I chose $5$ samples per request to create a continuous flow of data at 100 service requests per second. I thought that struck a good balance between, in the worst case, $4ms$ of latency while providing a continuous stream of data with no interruptions.
 
## Deploying
Any computer with a connected internet connection can simply clone the repository, extract desired packages, and place them in the computers ```src``` in a predefined ```colcon_ws```. Simply run ```colcon build``` in the root of the workspace and source your workspace.

Ensure that your sensor is connected to the computer via ethernet or is on the same network, with a defined IP that you can ping. If required, enable communication to occurr to the IP if your firewall prohibits unknown connections. 

## Runtime
The service server is a lifecycle node, meaning that its state can be configured programatically, or directly via the ROS2 CLI. Below is an image of the relevant states, and what important characteristics are present at each state:


## Grading Criteria
- We’re looking for code that is clean, readable, performant, and maintainable.
- The developer must think through the process of deploying and using the solution and provide the necessary documentation. 
- The sensor samples with 2000Hz, and you can request a specific number of samples in each call. Each call also has a ~1ms delay on top of the sampling time. We would like to hear your thought on picking the number of samples that you read in each call. 

## Submission
To submit the assignment, do the following:

1. Navigate to GitHub's project import page: [https://github.com/new/import](https://github.com/new/import)

2. In the box titled "Your old repository's clone URL", paste the homework repository's link: [https://github.com/Machina-Labs/robotic_hw](https://github.com/Machina-Labs/robotic_hw)

3. In the box titled "Repository Name", add a name for your local homework (ex. `Robotic_soln`)

4. Set the privacy level to "Public", then click "Begin Import" button at bottom of the page.

5. Develop your homework solution in the cloned repository and push it to GitHub when you're done. Extra points for good Git hygiene.

6. Send us the link to your repository.

## ROS2
Install instructions (Ubuntu): [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

ROS2 tutorials: [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)

