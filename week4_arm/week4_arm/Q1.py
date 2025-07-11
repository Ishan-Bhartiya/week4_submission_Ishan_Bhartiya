#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math

L1=2
L2=1.5
class MinimalSubscriber_cum_Publisher(Node):

    def __init__(self):
        super().__init__('Publisher_and_Subscriber')
        self.subscriber = self.create_subscription(JointState,'/joint_states',self.listener_callback,10)
        self.subscriber
        
        self.publisher= self.create_publisher(Point,'/end_effector_position',10)
        
    def listener_callback(self,msg):
        
        Message=Point()
        theta1=msg.position[1]+math.pi/2
        theta2=msg.position[2]
        x=L1*math.cos(theta1)+L2*math.cos(theta1+theta2)
        y=L1*math.sin(theta1)+L2*math.sin(theta1+theta2)
        Message.x=x
        Message.y=y       
        self.publisher.publish(Message)
        print('x : ', x, 'y :', y)
        
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber_cum_publisher=MinimalSubscriber_cum_Publisher()
    rclpy.spin(minimal_subscriber_cum_publisher)

if __name__ == '__main__':
    main()

