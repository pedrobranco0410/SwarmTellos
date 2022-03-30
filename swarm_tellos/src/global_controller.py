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


class GlobalController(Node):

    def __init__(self, names):
        #Initialization of the drone controller node
        super().__init__("global_controller")

        #Global Controller variables
        self.mode = 0
        self.base_position = [0,0,1.0,0]
        self.poly_angle = 0
        self.poly_angle_speed = 0.3
        self.in_position = False

        #Variables with information from the drones
        self.drones_name = names.copy()
        self.drones_state = [] #True/False
        self.drones_positions = [] #[x,y,z,rotation]
        self.drones_target_positions = [] #[x,y,z,rotation]

        #Clients Publishers and Subscribers
        self.drones_action_cli= []

        self.drones_target_positions_pub = []

        self.drones_state_sub = []
        self.drones_positions_sub = [] 

        self.action_srv = self.create_service(TelloAction, '/globalController/action', self.action_callback)

        #Timer and functions realated to time
        self.timer_period = 0.2 #seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


        #Declaring clients, subscribers and publisher for each drone
        for i in range(len(self.drones_name)):

            self.drones_state += [False]
            self.drones_positions += [[0,0,0,0]]
            self.drones_target_positions += [[0,0,0,0]]

            self.drones_action_cli += [self.create_client(TelloAction, '/'+ self.drones_name[i] +'_controller/tello_action')]

            self.drones_target_positions_pub += [self.create_publisher(Twist, '/' + self.drones_name[i] +'_controller/cmd_pos', 10)]

            self.drones_state_sub    += [self.create_subscription(State, '/'+ self.drones_name[i] +'_controller/state', self.status_callback, 10)]
            self.drones_positions_sub += [self.create_subscription(Position, '/'+ self.drones_name[i] +'_controller/pos', self.position_callback, 10)]


    ###########################
    #### CALLBACK FUNCTION ####
    ###########################
        
    def action_callback(self, request, response):

        req = TelloAction.Request()
        req.cmd = request.cmd
        self.in_position = False

        for i in range(len(self.drones_action_cli)):
            self.drones_action_cli[i].call_async(req)
        
        return response

    def status_callback(self, msg):
        old_status = self.drones_state.copy()
        for i in range(len(self.drones_name)):
             if(self.drones_name[i] == msg.drone_name):
                 self.drones_state[i] = msg.state

        if(old_status != self.drones_state):
            self.in_position = False

    def position_callback(self, msg):
         for i in range(len(self.drones_name)):
             if(self.drones_name[i] == msg.drone_name):
                 self.drones_positions[i][0] = msg.twist.linear.x
                 self.drones_positions[i][1] = msg.twist.linear.y
                 self.drones_positions[i][2] = msg.twist.linear.z
                 self.drones_positions[i][3] = msg.twist.angular.z
         
    def timer_callback(self):

        if(not self.in_position):
            self.go_to_formation()
        elif(self.mode == 0):
            self.poly_control()
        else:
            self.wave_control()
        self.send_drone_cmd()



    #############################
    #### PUBLISHER FUNCTIONS ####
    #############################
    
    def send_drone_cmd(self):
        '''
            This function will publish the target position for each drone in the system
        '''

        for i in range(len(self.drones_name)):
            twist = Twist()
            twist.linear.x = self.drones_target_positions[i][0] * 1.0
            twist.linear.y = self.drones_target_positions[i][1] * 1.0
            twist.linear.z = self.drones_target_positions[i][2] * 1.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.drones_target_positions[i][3] * 1.0

            self.drones_target_positions_pub[i].publish(twist)

    ###########################
    #### CONTROLLER SYSTEM ####
    ###########################

    def go_to_formation(self):

        #Checking how many drones exist in the system
        active_drones = sum(self.drones_state)
        ang = 2*math.pi/max(active_drones,1)

        #Calculating vertex positions in the polygon
        vertex = []
        for i in range(active_drones):
            vertex += [[self.base_position[0] + math.cos(ang*i +self.poly_angle),
                        self.base_position[1] + math.sin(ang*i +self.poly_angle),
                        self.base_position[2]]]

        #Assigning a vertex to each drone in the system 
        d_state = [0] * len(self.drones_state) #1 if the drone has a vertex 0 if not
        for j in range(active_drones):
            ind = 0
            dist = sys.float_info.max
            for i in range(len(self.drones_state)):
                if(self.drones_state[i] and not d_state[i] and dist > math.dist(vertex[j][:-1],self.drones_positions[i][:-2])):
                    ind = i
                    dist = math.dist(vertex[j],self.drones_positions[i][:-1])
            
            d_state[ind] = 1
            self.drones_target_positions[ind] = vertex[j]+[0]

                   
        #Checking if the drones have achieved their targets
        formation_xy = True
        formation_xyz = True
        for i in range(len(self.drones_state)):
            if(self.drones_state[i] and math.dist(self.drones_target_positions[i][:-2],self.drones_positions[i][:-2]) > 0.05):
                formation_xy = False
            if(self.drones_state[i] and math.dist(self.drones_target_positions[i][:-1],self.drones_positions[i][:-1]) > 0.05):
                formation_xyz = False

        #If all drones are in their target positions then the function has fulfilled its purpose
        if(formation_xyz):
            self.in_position = True
            return

        #If the drones are in their position in the xy-plane then all that is needed is to get the z-position right 
        #and no changes need to be made to the target position
        if(formation_xy):
            return

        
        #If there are drones out of position we need to make sure there are no collisions.
        #To do this we will have each one move one layer along the z-axis
        layer = True
        for i in range(6):
            if(self.drones_state[i] and math.dist([self.drones_positions[i][2]], [self.base_position[2]+ i*0.15]) > 0.02):
                layer = False
                break
        
        #If everyone is in the layer they can move around 
        for i in range(len(self.drones_state)):
            if(layer):
                self.drones_target_positions[i][2] = self.base_position[2] + i*0.15
            else:
                self.drones_target_positions[i] = [self.drones_positions[i][0],self.drones_positions[i][1],self.base_position[2] + i*0.15,0]
        
    def poly_control(self):
        
        #Checking how many drones exist in the system
        active_drones = sum(self.drones_state)
        ang = 2*math.pi/max(active_drones,1)

        #Calculating vertex positions in the polygon
        vertex = []
        for i in range(active_drones):
            vertex += [[self.base_position[0] + math.cos(ang*i +self.poly_angle),
                        self.base_position[1] + math.sin(ang*i +self.poly_angle),
                        self.base_position[2]]]

        #Assigning a vertex to each drone in the system 
        d_state = [0] * len(self.drones_state) #1 if the drone has a vertex 0 if not
        for j in range(active_drones):
            ind = 0
            dist = sys.float_info.max
            for i in range(len(self.drones_state)):
                if(self.drones_state[i] and not d_state[i] and dist > math.dist(vertex[j][:-1],self.drones_positions[i][:-2])):
                    ind = i
                    dist = math.dist(vertex[j],self.drones_positions[i][:-1])
            
            d_state[ind] = 1
            self.drones_target_positions[ind] = vertex[j]+[0]
                        
                        
        #Checking if the drones have achieved their targets
        total = 0
        for i in range(len(self.drones_state)):
             if(self.drones_state[i] and math.dist(self.drones_target_positions[i][:-1],self.drones_positions[i][:-1]) < 0.05):
                 total += 1
        
        if(total == active_drones):
            self.poly_angle += self.poly_angle_speed
            self.poly_angle = math.fmod(self.poly_angle, 2* math.pi)

    def wave_control(self): 
        #Checking how many drones exist in the system
        active_drones = sum(self.drones_state)
        ang = 2*math.pi/max(active_drones,1)
        ang_wave = math.pi/max(active_drones,1)

        #Calculating vertex positions in the polygon
        vertex = []
        for i in range(active_drones):

            base_wave = math.fmod(self.poly_angle + ang_wave*i, math.pi) 

            vertex += [[self.base_position[0] + math.cos(ang*i +self.poly_angle),
                        self.base_position[1] + math.sin(ang*i +self.poly_angle),
                        self.base_position[2] + math.sin(base_wave)]]

        #Assigning a vertex to each drone in the system 
        d_state = [0] * len(self.drones_state) #1 if the drone has a vertex 0 if not
        for j in range(active_drones):
            ind = 0
            dist = sys.float_info.max
            for i in range(len(self.drones_state)):
                if(self.drones_state[i] and not d_state[i] and dist > math.dist(vertex[j][:-1],self.drones_positions[i][:-2])):
                    ind = i
                    dist = math.dist(vertex[j],self.drones_positions[i][:-1])
            
            d_state[ind] = 1
            self.drones_target_positions[ind] = vertex[j]+[0]
                        
                        
        #Checking if the drones have achieved their targets
        total = 0
        for i in range(len(self.drones_state)):
             if(self.drones_state[i] and math.dist(self.drones_target_positions[i][:-1],self.drones_positions[i][:-1]) < 0.05):
                 total += 1
        
        if(total == active_drones):
            self.poly_angle += self.poly_angle_speed
            self.poly_angle = math.fmod(self.poly_angle, 2* math.pi)       


def main(args=None):

    if len(sys.argv) <= 2:
        print('usage: ros2 run swarm_tellos drone_controller.py -- drones_name')
        sys.exit(1)


    rclpy.init(args=args)
    

    globalController = GlobalController(sys.argv[1:-1])

    rclpy.spin(globalController)

    globalController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()