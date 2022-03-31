
# How to connect multiple dji tello edu to one network

First, would like to make some remarks:
	1. It does not work for dji tello, only for the dji tello edu. As only the second one has the SDK mode available and permits to connect the drone direct to a network.
	2. It is important to connect the drones to a network which you can configure the IP adress of the drones.

## 1) Install packetsender

In one terminal: 
	sudo apt install packetsender
	
## 2) Configure the tello to connect in a network

	1. Connect to the tello wifi
	
	2. Open the packetsender and send an UDP packet:
		name: <free to choose>
		ASCII: command
		address: 192.168.10.1
		port: 8889
		Resend Delay: 0
		UDP
	
	3. Send another UDP packet:
		name: <free to choose>
		ASCII: ap <network_name> <network_password>
		address: 192.168.10.1
		port: 8889
		Resend Delay: 0
		UDP
	
	4. Connect to the network and set an static ip for the tello
		
		In this case we used a router. For more info on how to set an static ip address consult the manual of your router.
		
	
	Repeat this process for all the drones you want to connect. After this configuration, every time you turn on your tello it will automaticaly connect to the wifi you have set it and you can send commands to it by just changing the ip address. In case you want to reset the tello to the default configuration just press the on/off button for 5 seconds and it will be restored.



# Instructions to stream mocap_optitrack node from Ros Noetic to Ros2 foxy

## 1) Install Ros Noetic

[Link to install Ros Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

## 2) Install Ros2 foxy

[Link to install Ros2 Foxy](http://wiki.ros.org/noetic/Installation/Ubuntu)

## 3) Install RosBridge package in Ros2

In one terminal:	
	source /opt/ros/foxy/setup.bash
	sudo apt-get update
	sudo apt-get install ros-foxy-ros1-bridge
	
## 4) Install mocap_optitrack package

In one terminal:
	source /opt/ros/noetic/setup.bash
	sudo apt-get update
	sudo apt-get install ros-noetic-mocap-optitrack

## 5) Stream the nodes

First terminal:
	source /opt/ros/noetic/setup.bash
	roscore

Second terminal:
	source /opt/ros/noetic/setup.bash
	roslaunch mocap_optitrack mocap.launch
	
third terminal:
	source /opt/ros/foxy/setup.bash
	source /opt/ros/noetic/setup.bash
	ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
