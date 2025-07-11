Package : week4_arm
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Node Q1.py : This node is responsible for Subscribing to the node called 'joint_states', RVIZ simulator publishes to this node, three positions(which described in Q1.py)
described in meters or radians. It then prints the message received to the console, uses forward kinematics to find the cartesian coordinate of the end effector and then
publishes this message to the topic called 'end_effector_position' using Point datatype

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Node Q3.py : This node is responsible for subscribing to the node called 'end_effector_positions'. It takes the current cartesian coordinates of the end effector as 
input, then takes input using terminal from the user, regarding whether the user wants to move the end effector along the x or y axis, and if so, then by how much.
If the manoeuvre is possible, it calculates the final position, and then calculates the joint's angle with respect to the ground and each other using inverse kinematics.
Finally, it publishes the angles using Float64MultiArray datatype to the 'joint_angles_goal' topic.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
