#!/usr/bin/env python3

import rclpy
import sys
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tello_msgs.msg import FlightData
from tello_msgs.msg import TelloResponse
from tello_msgs.srv import TelloAction

from sensor_msgs.msg import NavSatFix

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

        #Defining the topics where the controller will publish messages
        self.speed_pub = self.create_publisher(Twist, '/' + self.drone_name +'/cmd_vel', 1)
        self.state_pub = self.create_publisher(Bool, '/' + self.namespace +'/state', 10)
        self.posi_pub  = self.create_publisher(Twist, '/' + self.namespace +'/pos', 10)

        #Defining the data that will be received by the controller and the respective topics
        self.imu_sub   = self.create_subscription(FlightData,'/' + self.drone_name + '/flight_data',self.imu_callback,10)
        self.gps_sub   = self.create_subscription(NavSatFix,'/drone_' + self.drone_name[-1] + '/gps',self.gps_callback,10)
        self.cmd_sub   = self.create_subscription(Twist,'/' + self.namespace +'/cmd_pos',self.cmd_callback,10)

        #Defining Services and Clients
        self.action_cli = self.create_client(TelloAction, '/' + self.drone_name  +'/tello_action')
        self.action_srv = self.create_service(TelloAction, '/' + self.namespace +'/tello_action', self.action_callback)

        #Setting a timer that will be responsible for performing all the drone control procedures. 
        self.timer_period = 0.1 #seconds
        self.time_counter = 0
        self.max_wait = 5 #seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        #Setting PID Controllers
        self.pid_linear  = PID(0.2, 0.0, 0.0, setpoint=0)
        self.pid_linear.sample_time = self.timer_period
        self.pid_linear.output_limits = (-1, 0)

        self.pid_z  = PID(0.5, 0.01, 0.0, setpoint=0)
        self.pid_z.sample_time = self.timer_period
        self.pid_z.output_limits = (-1, 1)

        self.pid_angular  = PID(0.5, 0, 0, setpoint=0)
        self.pid_angular.sample_time = self.timer_period
        self.pid_angular.output_limits = (-1, 1)
        

    ###########################
    #### CALLBACK FUNCTION ####
    ###########################

    def imu_callback(self, msg):
        #If we receive information from the drone it means that the drone is active and therefore we need to update the state variables
        if(self.state == 0 and self.time_counter == self.max_wait): #If the drone was inactive
            self.get_logger().info('Contact with "%s" has been reestablished and its controller is reactivated' % self.drone_name)
        
        if(self.state != 1):
            self.state = 2
        self.time_counter = 0
        self.imu_data = msg
        
    def gps_callback(self, msg):
        self.gps_data = msg

    def cmd_callback(self, msg):

        self.target_position[0] = msg.linear.x
        self.target_position[1] = msg.linear.y
        self.target_position[2] = msg.linear.z
        self.target_position[3] = msg.angular.z

        self.pid_z.setpoint = self.target_position[2]
        self.pid_angular.setpoint = self.target_position[3]
        
    def action_callback(self, request, response):

        if(request.cmd == 'takeoff'):
            self.state = 2           
            self.send_action(request.cmd)

        elif(request.cmd == "land"):
            self.state = 1
            self.speed = [0,0,0,0]
            self.send_action(request.cmd)
        
        return response

    def timer_callback(self):

        #Checking if contact with the drone has been lost
        if(self.state == 0 and self.time_counter > self.max_wait):
            self.send_state(False)
            self.time_counter = self.max_wait
            self.get_logger().info('Contact with "%s" has been lost and its controller has been deactivated' % self.drone_name)

        elif(self.time_counter == self.max_wait):
            self.send_action("takeoff")
            self.send_state(False)

        if(self.state == 2):
            self.update_position ()
            self.update_speed ()

            #Incrementing the time counter
            #self.time_counter += self.timer_period
            #self.state = 0

            #Sending controller information and commands
            self.send_state(True)
            self.send_drone_position()
            self.send_speed_cmd()
        else:
            self.send_state(False)
            self.update_position ()
            self.send_drone_position()

    ###########################################################################
    #### Functions responsible for sending information from the controller ####
    ########################################################################### 

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

        twist=Twist()
        twist.linear.x = self.position[0] * 1.0
        twist.linear.y = self.position[1] * 1.0
        twist.linear.z = self.position[2] * 1.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.position[3] * 1.0
        self.posi_pub.publish(twist)

    def send_state(self, actual_state):
        '''
            Function responsible for publishing the state of the controller. The state will be True 
            when the controller has communication with the drone and False if the controller has not 
            received any information from the drone in the last "max_wait" seconds
        '''

        msg = Bool()
        msg.data = actual_state
        self.state_pub.publish(msg)

    def send_action(self, action):
        req = TelloAction.Request()
        req.cmd = 'takeoff'
        self.action_cli.call_async(req)
        
    ############################################################
    #### Functions responsible for the drone control system ####
    ############################################################

    def update_position(self):
        if(self.gps_data != None):
            self.position[0] = -self.gps_data.longitude * 111000
            self.position[1] = -self.gps_data.latitude * 111000
            self.position[2] = self.gps_data.altitude
            #self.position[3] = self.gps_data.angular.z
            self.gps_data = None
        
        else:
            self.position[0] += (self.speed[0]*math.cos(self.position[3]) + self.speed[1]*math.cos(self.position[3] - math.pi/2))* self.timer_period
            self.position[1] -= (self.speed[0]*math.sin(self.position[3]) + self.speed[1]*math.sin(self.position[3] - math.pi/2))* self.timer_period
            self.position[2] += self.speed[2] * self.timer_period
            self.position[3] += self.speed[3] * self.timer_period


    def update_speed(self):

        #
        self.speed[2] = self.pid_z(self.position[2])

        linear_dist = math.dist([self.position[0],self.position[1]],[self.target_position[0], self.target_position[1]])
        ang = math.atan2(self.target_position[1] - self.position[1],self.target_position[0] - self.position[0]) + self.position[3]

        angular_dist = math.dist([self.position[3]], [self.target_position[3]])

        if(linear_dist <= 0.05):
            self.speed[0] = 0.0
            self.speed[1] = 0.0
            self.speed[3] = self.pid_angular(self.position[3])
        else:
            l_speed = -self.pid_linear(linear_dist)
            self.speed[0] = l_speed * math.cos(ang)
            self.speed[1] = l_speed * math.sin(ang)

        


            
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