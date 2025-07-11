#!/usr/bin/env python3 #Shebang for python files
#Importing necessary modules and libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import math

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('Inverse_Kinematics_Node')#Node has been named 'Inverse_Kinematics_Node'
        self.subscriber=self.create_subscription(Point,'/end_effector_position',self.listener_callback,10)#Creating subscriber object 
        self.subscriber#Prevents unused variable warnings
        
        self.publisher=self.create_publisher(Float64MultiArray,'/joint_angles_goal',10)#Creates publisher object , publishes on joint_angles_goal topic with the allowed message type of Float64MultiArray
        
    def listener_callback(self,msg):
    
        Message=Float64MultiArray()#Float64MultiArray type object created
        xory=input("Enter 'x' or 'y' if you want the end effector to move in those directions : ")#Takes input from user on whether he/she wants to move on the x axis or y axis
        dist=float(input("Enter distance it moves, given that it is less than 0.5m and more than -0.5m : "))
        X=msg.x#Using dummy variables instead of the received message's parameters
        Y=msg.y
        if abs(dist)<=0.5:
            if xory=='x':
                if ((X+dist)**2+Y**2-6.25)/6<=1:#Makes sure to consider only those values of dist due to which the argument of the cos function doesn't go out of the domain i.e [-1,1]
                    X+=dist   
                    print(X,Y)
                else:
                    print("This value of x leads the robot out of its boundary")#Handles cases where the argument of the cos function goes out of the domain
            elif xory=='y':
                if (X**2+(Y+dist)**2-6.25)/6<=1:#Makes sure to consider only those values of dist due to which the argument of the cos function doesn't go out of the domain i.e [-1,1]
                    Y+=dist
                    print(X,Y)
                else:
                    print("This value of y leads the robot out of its boundary")#Handles cases where the argument of the cos function goes out of the domain
            else:
                pass#ignores and proceeds with the execution if the input is neither x nor y
        else:
            print("Distance cannot be more than 0.5m or lesser than -0.5m")
            
        '''These can be calculated by using inverse kinematics, if we use the equations for x and y from Q1.py, square each of them and add them, we can rewrite it to give the expression  of
        theta_2, and theta_1 can be found using some simple geometry involving theta_2'''
        
        theta_2=math.acos(((X)**2+(Y)**2-6.25)/6)
        theta_1=math.atan(Y/X)-math.atan(1.5*math.sin(theta_2)/(2+1.5*math.cos(theta_2)))
        Message.data=[theta_1,theta_2]
        self.publisher.publish(Message)#Publishes the message
        
def main(args=None):
    rclpy.init(args=args)#Initialises rclpy Python library
    inverse_kinematics=InverseKinematics()#Createsan instance of the MinimalSubscriber_cum_Publisher class
    rclpy.spin(inverse_kinematics)#Keeps running the above instance unless stopped

if __name__ == '__main__':#Sets __name__ to '__main__ if the file is run directly, and not if it is imported as amodule into another script
    main()                #Doing so helps running the file only if it is called to be run
