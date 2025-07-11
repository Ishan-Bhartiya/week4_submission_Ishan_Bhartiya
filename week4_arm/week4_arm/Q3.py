#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import math

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('Inverse_Kinematics_Node')
        self.subscriber=self.create_subscription(Point,'/end_effector_position',self.listener_callback,10)
        self.subscriber
        
        self.publisher=self.create_publisher(Float64MultiArray,'/joint_angles_goal',10)
        
    def listener_callback(self,msg):
    
        Message=Float64MultiArray()
        xory=input("Enter 'x' or 'y' if you want the end effector to move in those directions : ")
        dist=float(input("Enter distance it moves, given that it is less than 0.5m and more than -0.5m : "))
        X=msg.x
        Y=msg.y
        if abs(dist)<=0.5:
            if xory=='x':
                if ((X+dist)**2+Y**2-6.25)/6<=1:
                    X+=dist   
                    print(X,Y)
                else:
                    print("This value of x leads the robot out of its boundary")
            elif xory=='y':
                if (X**2+(Y+dist)**2-6.25)/6<=1:
                    Y+=dist
                    print(X,Y)
                else:
                    print("This value of y leads the robot out of its boundary")
            else:
                pass
        else:
            print("Distance cannot be more than 0.5m or lesser than -0.5m")
            
        theta_2=math.acos(((X)**2+(Y)**2-6.25)/6)
        theta_1=math.atan(Y/X)-math.atan(1.5*math.sin(theta_2)/(2+1.5*math.cos(theta_2)))
        Message.data=[theta_1,theta_2]
        self.publisher.publish(Message)
        
def main(args=None):
    rclpy.init(args=args)
    inverse_kinematics=InverseKinematics()
    rclpy.spin(inverse_kinematics)

if __name__ == '__main__':
    main()
