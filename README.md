# ROS Delay Checker
This ROS package checks the delay between topics.

Installation
To install the package, follow these steps:

```shell
$ cd ~/catkin_ws/src
$ git clone https://github.com/ChanJoon/ros_delay_checker.git
$ cd ~/catkin_ws && catkin_make
```

## Usage
To use the package, follow these steps:

1. Launch your communication node, which converts the message type "geometry_msgs::Twist" to the ERP42 protocol.

2. Run the delay_checker_node:

	```shell
	# $ roslaunch <your communication node>
	$ rosrun delay_checker delay_checker_node
	```

3. Publish "/cmd_vel" using the keyboard.

4. Wait for the package to subscribe to "/output".

5. Check your delay.