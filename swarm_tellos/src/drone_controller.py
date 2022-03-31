#!/usr/bin/env python3

import rclpy
import sys
import math

from rclpy.node import Node

from geometry_msgs.msg import Twist

from tello_msgs.msg import TelloResponse
from tello_msgs.srv import TelloAction

from swarm_msgs.msg import State
from swarm_msgs.msg import Position

from geometry_msgs.msg import PoseStamped
#from sensor_msgs.msg import Imu

from simple_pid import PID


class DroneController(Node):

    def __init__(self, name):
        
        #Name of the drone we are controlling
        self.drone_name = name
        self.namespace = self.drone_name + "_controller"

        #Variables that store the drone's information
        self.position = [0,0,0,0]  #x,y,z,rotation
        self.target_position = [0,0,0,0]#x,y,z,rotation
        self.speed = [0,0,0,0] #x,y,z,rotation
        self.state = 1 # 0: no communication, 1:landed, 2:flying with comunication 

        #variables that hold the received information
        self.gps_data = None
        self.imu_data = None

        #Initialization of the drone controller node
        super().__init__(self.namespace)


        '''Setting Publishers'''
        self.speed_pub = self.create_publisher(Twist, '/' + self.drone_name +'/cmd_vel', 1)
        self.state_pub = self.create_publisher(State, '/' + self.namespace +'/state', 10)
        self.posi_pub  = self.create_publisher(Position, '/' + self.namespace +'/pos', 10)

        '''Setting Subscribers'''
        #self.imu_sub   = self.create_subscription(Imu,'/drone_' + self.drone_name[5:] + '/imu',self.imu_callback,10)
        self.gps_sub   = self.create_subscription(PoseStamped,'/mocap_node/drone_' + self.drone_name[5:] + '/gps',self.gps_callback,10)
        self.cmd_sub   = self.create_subscription(Twist,'/' + self.namespace +'/cmd_pos',self.cmd_callback,10)

        '''Setting Services and clients'''
        self.action_cli = self.create_client(TelloAction, '/' + self.drone_name  +'/tello_action')
        self.action_srv = self.create_service(TelloAction, '/' + self.namespace +'/tello_action', self.action_callback)

        '''Setting time and time variables'''
        self.timer_period = 0.1 #seconds
        self.time_counter = 0
        self.max_wait = 5 #seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


        '''Setting PID Controllers'''
        #A PID to control the linear distance(x,y)
        self.pid_linear  = PID(1.0, 0.0, 0.0, setpoint=0)
        self.pid_linear.sample_time = self.timer_period
        self.pid_linear.output_limits = (-1, 0)

        #A PID to control altitude
        self.pid_z  = PID(1.0, 0.0, 0.0, setpoint=0)
        self.pid_z.sample_time = self.timer_period
        self.pid_z.output_limits = (-1, 1)

        #A PID to control the drone's rotation
        self.pid_angular  = PID(0.8, 0, 0, setpoint=0)
        self.pid_angular.sample_time = self.timer_period
        self.pid_angular.output_limits = (-1, 1)
        

    ###########################
    #### CALLBACK FUNCTION ####
    ###########################

    def imu_callback(self, msg):

        #If we receive information from the drone it means that the drone is active and therefore we need to update the state variables
        if(self.state == 0 and self.time_counter == self.max_wait): #If the drone was inactive
            self.get_logger().info('Contact with "%s" has been reestablished and its controller is reactivated' % self.drone_name)
        
        #If the drone is not on the ground we will change status to flying with communication
        if(self.state != 1):
            self.state = 2


        self.time_counter = 0
        self.imu_data = msg
        
    def gps_callback(self, msg):

        #If we receive information from the drone it means that the drone is active and therefore we need to update the state variables
        if(self.state == 0 and self.time_counter == self.max_wait): #If the drone was inactive
            self.get_logger().info('Contact with "%s" has been reestablished and its controller is reactivated' % self.drone_name)
        
        #If the drone is not on the ground we will change status to flying with communication
        if(self.state != 1):
            self.state = 2

        self.time_counter = 0
        self.gps_data = msg

    def cmd_callback(self, msg):
        """
            When we receive a new command from the master controller we will update the target position
        """

        self.target_position[0] = msg.linear.x
        self.target_position[1] = msg.linear.y
        self.target_position[2] = msg.linear.z
        self.target_position[3] = msg.angular.z

        #We will also update the PID setpoints
        self.pid_z.setpoint = self.target_position[2]
        self.pid_angular.setpoint = self.target_position[3]
        
    def action_callback(self, request, response):
        """
            Function that will be called to send actions to the drone
        """

        #If the drone takes off we will change the landed status to flying without communication
        if(request.cmd == 'takeoff'):
            self.state = 0          
            self.send_action(request.cmd)

        #If the drone lands, we will set its speed to zero and change its status
        elif(request.cmd == "land"):
            self.state = 1
            self.speed = [0,0,0,0]
            self.send_action(request.cmd)
        
        return response

    def timer_callback(self):

        '''
        This function will be called by the controller every timer_period seconds and will be responsible for performing all c
        ontroller operations. Check the drone status, update the drone position and calculate the speed commands.
        '''

        if(self.state == 0 and self.time_counter > self.max_wait):
            #If contact with the drone has been lost we will tell it to land and set the timer count to the maximum wait value
            self.send_state(False)
            self.time_counter = self.max_wait
            self.send_action("land")
            self.get_logger().info('Contact with "%s" has been lost and its controller has been deactivated' % self.drone_name)

        elif(self.state == 0 and self.time_counter == self.max_wait):
            self.speed = [0,0,0,0]
            self.send_speed_cmd()

        elif(self.state == 0):
            #If the drone is receiving no data, no communication, the drone must stay still and not perform any movement until it has recovered data or reached the maximum waiting time
            self.time_counter += self.timer_period
            self.speed = [0,0,0,0]
            self.send_speed_cmd()

        elif(self.state == 1):
            #If the drone is landed it will only update the position
            self.send_state(False)
            self.update_position ()
            self.send_drone_position()

        elif(self.state == 2):

            #If the drone is communicating well we will update its position and calculate the control signal
            self.update_position ()
            self.update_speed ()

            #Incrementing the time counter
            self.time_counter += self.timer_period
            self.state = 0

            #Sending controller information and commands
            self.send_state(True)
            self.send_drone_position()
            self.send_speed_cmd()
        
        

    #############################
    #### PUBLISHER FUNCTIONS ####
    #############################

    def send_speed_cmd(self):
        '''
            This function will send the calculated speed command based on the PID to the drone
        '''
        twist=Twist()
        twist.linear.x = self.speed[0] * 1.0
        twist.linear.y = self.speed[1] * 1.0
        twist.linear.z = self.speed[2] * 1.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.speed[3] * 1.0
        self.speed_pub.publish(twist)

    def send_drone_position(self):
        '''
            This function will publish the drone's position calculated from the inertial center and the 
            gps data (in the project we used a camera system to simulate the gps)
        '''

        msg=Position()
        msg.drone_name = self.drone_name
        msg.twist.linear.x = self.position[0] * 1.0
        msg.twist.linear.y = self.position[1] * 1.0
        msg.twist.linear.z = self.position[2] * 1.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = self.position[3] * 1.0
        self.posi_pub.publish(msg)

    def send_state(self, actual_state):
        '''
            Function responsible for publishing the state of the controller. The state will be True 
            when the controller has communication with the drone and False if the controller has not 
            received any information from the drone in the last "max_wait" seconds
        '''

        msg = State()
        msg.drone_name = self.drone_name
        msg.state = actual_state
        self.state_pub.publish(msg)

    def send_action(self, action):
        req = TelloAction.Request()
        req.cmd = action
        self.action_cli.call_async(req)
        


    ######################
    #### DRONE SYSTEM ####
    ######################

    def update_position(self):
        """
            Function responsible for performing data fusion between the available information and calculating the drone's real position. 
            As in the real system we have a set of cameras that give us the exact position we don't develop any specific function and we 
            directly get the position calculated by the system that is published in the gps topic. However, functions such as a kalman filter 
            can be developed that perform data fusion in a more robust way.We did not use the data from imu because for technical reasons it 
            was not possible to retrieve them in a multi-drones system all connected to the same networkSince we also did not add imu to the 
            simulation we were also unable to retrieve the data in the simulated environment. To simulate the camera system we added a gps to 
            the drone which gives us a position with a good confidence margin.. 
        """


        if(self.gps_data != None):
            self.position[0] = self.gps_data.pose.position.y
            self.position[1] = -self.gps_data.pose.position.x
            self.position[2] = self.gps_data.pose.position.z
            self.position[3] = math.atan2(
                (2 * (self.gps_data.pose.orientation.w * self.gps_data.pose.orientation.z + self.gps_data.pose.orientation.x*self.gps_data.pose.orientation.y)),
                (1 - 2 *(self.gps_data.pose.orientation.y*self.gps_data.pose.orientation.y + self.gps_data.pose.orientation.z*self.gps_data.pose.orientation.z)))
            
            self.gps_data = None


        
    def update_speed(self):
        '''
            This function is responsible for calculating the control signals (speeds) based on the current state and the goal state we want to achieve
        '''

        #Calculating altitude control commands
        self.speed[2] = self.pid_z(self.position[2])

                
        #Calculating distance and angle between current position and target position  
        linear_dist = math.dist([self.position[0],self.position[1]],[self.target_position[0], self.target_position[1]])
        ang = math.atan2(self.target_position[1] - self.position[1],self.target_position[0] - self.position[0]) + self.position[3]


        #Setting a control threshold. If the drone is within a 5 cm radius of its target position we will consider it to be at the target position
        if(math.dist([self.position[3]] , [self.target_position[3]]) >  0.1):
            #If it is in a good position you should stop and adjust the rotation
            self.speed[0] = 0.0
            self.speed[1] = 0.0
            self.speed[3] = self.pid_angular(self.position[3])
        
        else:
            #If it is far from its target position we should calculate the speed commands and not modify its rotation
            l_speed = -self.pid_linear(linear_dist)
            self.speed[0] = l_speed * math.cos(ang)
            self.speed[1] = l_speed * math.sin(ang)
            self.speed[3] = 0.0

        if(self.speed[0] > 0.02 and self.speed[0] < 0.1):
            self.speed[0] = 0.1
        if(self.speed[1] > 0.02 and self.speed[1] < 0.1):
            self.speed[1] = 0.1
        if(self.speed[2] > 0.02 and self.speed[2] < 0.1):
            self.speed[2] = 0.1
        if(self.speed[3] > 0.02 and self.speed[3] < 0.1):
            self.speed[3] = 0.1

        if(self.speed[0] < -0.02 and self.speed[0] > -0.1):
            self.speed[0] = -0.1

        if(self.speed[1] < -0.02 and self.speed[1] > -0.1):
            self.speed[1] = -0.1

        if(self.speed[2] < -0.02 and self.speed[2] > -0.1):
            self.speed[2] = -0.1

        if(self.speed[3] < -0.02 and self.speed[3] > -0.1):
            self.speed[3] = -0.1




def main(args=None):

    if len(sys.argv) < 2:
        print('usage: ros2 run swarm_tellos drone_controller.py -- namespace')
        sys.exit(1)


    rclpy.init(args=args)

    droneController = DroneController(str(sys.argv[1]))

    rclpy.spin(droneController)

    dronController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()