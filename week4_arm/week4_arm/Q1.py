#!/usr/bin/env python3 #Shebang for python files
#Importing necessary modules and libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math

L1=2
L2=1.5
class MinimalSubscriber_cum_Publisher(Node):

    def __init__(self):
        super().__init__('Publisher_and_Subscriber') #Naming the node as Publisher_and_Subscriber
        self.subscriber = self.create_subscription(JointState,'/joint_states',self.listener_callback,10)#Creating Subscriber Object, listens to what is published on the topic joint_states by Rviz
        self.subscriber #Prevents warnings on unused variables
        
        self.publisher= self.create_publisher(Point,'/end_effector_position',10) #Creates publisher object, which publishes String type message on the topic end_effector_position
        
    def listener_callback(self,msg):
        
        Message=Point() #Creating Point object
        '''the position field of themessage sent by rviz,is a list of 3 elements,the first (base_yaw_joint) is the angle between the z axis and the base rod, 2nd (shoulder_pitch_joint) is the
        angle between the base rod and its projection on the ground plane i.e theta1,3rd(elbow_joint) is the angle between base rod and the rod connected to the end effector i.e theta2'''
        theta1=msg.position[1]+math.pi/2 
        theta2=msg.position[2]

        ''' x and y can be derived using forward kinematics, either using transformation matrices or simple geometry(with the correct diagram) '''
        x=L1*math.cos(theta1)+L2*math.cos(theta1+theta2)
        y=L1*math.sin(theta1)+L2*math.sin(theta1+theta2)
        Message.x=x #Setting the x field of Message to the above calculated x
        Message.y=y #Setting the x field of Message to the above calculated x
        self.publisher.publish(Message)#Publishes Message
        print('x : ', x, 'y :', y)
        
def main(args=None):
    rclpy.init(args=args)#Initialises rclpy Python library
    minimal_subscriber_cum_publisher=MinimalSubscriber_cum_Publisher()#Createsan instance of the MinimalSubscriber_cum_Publisher class
    rclpy.spin(minimal_subscriber_cum_publisher)# Keeps running the above instance unless stopped

if __name__ == '__main__':#Sets __name__ to '__main__ if the file is run directly, and not if it is imported as amodule into another script
    main()                #Doing so helps running the file only if it is called to be run

