# Lab 1: Intro to ROS 2

## Written Questions

### Q1: During this assignment, you've probably ran these two following commands at some point: ```source /opt/ros/jazzy/setup.bash``` and ```source install/local_setup.bash```. Functionally what is the difference between the two?

Answer: “source /opt/ros/jazzy/setup.bash“ is used to load the system-installed ROS 2 Jazzy environment, allowing the terminal to access ROS 2 commands and system packages.
“source install/local_setup.bash“ is used to load the packages built in the current workspace, such as lab1_pkg.


### Q2: What does the ```queue_size``` argument control when creating a subscriber or a publisher? How does different ```queue_size``` affect how messages are handled?

Answer: queue_size controls the maximum number of unprocessed messages that a publisher or subscriber can buffer.A smaller queue uses less memory and usually results in lower latency, but messages are more likely to be dropped if they are produced too quickly. A larger queue can store more messages and better handle short bursts of traffic, but it consumes more memory and may cause the subscriber to process older messages, which can increase latency.


### Q3: Do you have to call ```colcon build``` again after you've changed a launch file in your package? (Hint: consider two cases: calling ```ros2 launch``` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

Answer: If the launch file is executed directly from the source directory, modifications take effect immediately and rebuilding is not required.However, if the launch file is executed using “ros2 launch lab1_pkg lab1_launch.py“, which runs the launch file installed inside the package, you need to run “colcon build“ again so that the updated file is copied to the install directory.

