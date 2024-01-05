#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import sin, cos, pi
import random 
from planar_3dof_control.msg import EndEffector
from geometry_msgs.msg import Vector3
from math import atan2, pi, sin, cos, pow, sqrt, acos



"""
Topics To Write on:
type: std_msgs/Float64
/antropomorphic_arm/joint1_position_controller/command
/antropomorphic_arm/joint2_position_controller/command
/antropomorphic_arm/joint3_position_controller/command

"""
from dataclasses import dataclass
@dataclass

class JointMover(object):

    def __init__(self):

        rospy.loginfo("JointMover Initialising...")
        self.pub_joint1 = rospy.Publisher('/antropomorphic_arm/joint1_position_controller/command',
                                                            Float64,
                                                            queue_size=1)
        self.pub_joint2 = rospy.Publisher('/antropomorphic_arm/joint2_position_controller/command',
                                                             Float64,
                                                             queue_size=1)
        self.pub_joint3 = rospy.Publisher('/antropomorphic_arm/joint3_position_controller/command',
                                                           Float64,
                                                           queue_size=1)

        self.check_connection()

    def move_all_joints(self, theta_1, theta_2, theta_3):
        theta_1_msg = Float64()
        theta_1_msg.data = theta_1
        theta_2_msg = Float64()
        theta_2_msg.data = theta_2
        theta_3_msg = Float64()
        theta_3_msg.data = theta_3
        self.pub_joint1.publish(theta_1_msg)
        self.pub_joint2.publish(theta_2_msg)
        self.pub_joint3.publish(theta_3_msg)

        rospy.logwarn("Moving to Angles= theta_1="+str(theta_1)+", theta_2="+str(theta_2)+", theta_3="+str(theta_3))


    def check_topic_connection(self, publisher_object, topic_name):

        self.rate = rospy.Rate(10)  # 10hz
        conections_joint_1 = publisher_object.get_num_connections()
        rospy.logdebug(conections_joint_1)
        while conections_joint_1 == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to "+str(topic_name)+" yet so we wait and try again")

            conections_joint_1 = publisher_object.get_num_connections()

            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                rospy.logdebug("ROSInterruptException Triggered")
            

        rospy.logdebug(str(topic_name)+" Publisher Connected")

    def check_connection(self):
        """
        Checks publisher is working
        :return:
        """
        self.check_topic_connection(self.pub_joint1, "pub_joint1")
        self.check_topic_connection(self.pub_joint2, "pub_joint2")
        self.check_topic_connection(self.pub_joint3, "pub_joint3")

        rospy.logdebug("All Publishers READY")
 

def ee_command_callback(msg, obj):
    # Extracting data from the received message
    x = msg.ee_xy_theta.x
    y = msg.ee_xy_theta.y
    z = msg.ee_xy_theta.z
    config = msg.elbow_policy.data
    r1 = 0.0
    r2 = 1.0
    r3 = 1.0

    DH_parameters={"r1":r1,
                    "r2":r2,
                    "r3":r3}

    thetas, possible_solution = calculate_ik(Pee_x=x, Pee_y=y, Pee_z=z, DH_parameters=DH_parameters, elbow_config= config)
    obj.move_all_joints(thetas[0],thetas[1],thetas[2])



class EndEffectorWorkingSpace:
    # Pos of The P2, which is the one we resolved for
    Pee_x: float
    Pee_y: float
    Pee_z: float

class ComputeIk():

    def __init__(self, DH_parameters):
        
        # DH parameters
        self.DH_parameters_ = DH_parameters

    def get_dh_param(self, name):

        if name in self.DH_parameters_:
            return self.DH_parameters_[name]
        else:
            assert False, "Asked for Non existen param DH name ="+str(name)

    def compute_ik(self, Pee_x, Pee_y, Pee_z, elbow_configuration = "plus-plus"):
        
        # Initialization
        # Pee_x = end_effector_pose.Pee_x
        # Pee_y = end_effector_pose.Pee_y
        # Pee_z = end_effector_pose.Pee_z

        # We get all the DH parameters
        r1 = self.get_dh_param("r1")
        r2 = self.get_dh_param("r2")
        r3 = self.get_dh_param("r3")

        # We declare all the equations for theta1, theta2, theta3 and auxiliary
        #########################################################################


        ## WE HAVE TO CHECK THAT ITS POSSIBLE
        possible_solution = True


        theta_array = []

        G = 0.5*(-2 + pow(Pee_x,2) + pow(Pee_y,2) + pow(Pee_z,2))

        if elbow_configuration == "plus-plus":
            theta_1 = atan2(Pee_y,Pee_x)
            theta_3 = acos(G)
            theta_2 = acos(Pee_y/(2*sin(theta_1)*cos(theta_3*0.5)))-theta_3*0.5

        elif elbow_configuration == "plus-minus":
            theta_1 = atan2(Pee_y,Pee_x)
            theta_3 = -acos(G)
            theta_2 = acos(Pee_y/(2*sin(theta_1)*cos(theta_3*0.5)))-theta_3*0.5

        elif elbow_configuration == "minus-plus":
                theta_1 = atan2(Pee_y,Pee_x)-pi
                theta_3 = acos(G)
                theta_2 = acos(Pee_y/(2*sin(theta_1)*cos(theta_3*0.5)))-theta_3*0.5
        else :
            theta_1 = atan2(Pee_y,Pee_x)-pi
            theta_3 = -acos(G)
            theta_2 = acos(Pee_y/(2*sin(theta_1)*cos(theta_3*0.5)))-theta_3*0.5

        if theta_3 <= pi*-0.75 or theta_3 >= pi*0.75:
            possible_solution = False
        if theta_2 <= pi*-0.25 or theta_2 >= pi*0.75:
            possible_solution = False

        theta_array = [theta_1, theta_2, theta_3]

        return theta_array, possible_solution

def calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, elbow_config = "plus-plus"):

    ik = ComputeIk(DH_parameters = DH_parameters)

    thetas, possible_solution = ik.compute_ik(  Pee_x = Pee_x, Pee_y = Pee_y, Pee_z = Pee_z,
                                                elbow_configuration = elbow_config,
                                                )

    print("("+str(thetas)+"," + str(possible_solution) + ")\n")
    return thetas, possible_solution


if __name__ == "__main__":
    rospy.init_node('end_effector_move',log_level=rospy.DEBUG)
    obj = JointMover()
    rospy.Subscriber('/ee_pose_commands', EndEffector, ee_command_callback, obj)

    rospy.spin()

