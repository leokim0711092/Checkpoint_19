#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import sin, cos, pi
import random 
from geometry_msgs.msg import Vector3
from math import atan2, pi, sin, cos, pow, sqrt, acos


def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


class MarkerBasics(object):

    def __init__(self):
        self.marker_objectlisher = rospy.Publisher('/ee_position', Marker, queue_size=1)
        self.end_pose_sub = rospy.Subscriber('/end_effector_real_pose', Vector3, self.end_effector_callback)

        self.init_marker(index=0)
        self.index = 0
    
    def end_effector_callback(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        
        rospy.loginfo("Marker x: %f", x)
        rospy.loginfo("Marker y: %f", y)
        rospy.loginfo("Marker z: %f", z)

        self.publish_point(x,y,z,self.index)
        self.index += 1



    def init_marker(self,index=0):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = "frame_0"
        self.marker_object.header.stamp    = rospy.get_rostime()
        self.marker_object.ns = ""
        self.marker_object.id = index
        self.marker_object.type = Marker.SPHERE
        self.marker_object.action = Marker.ADD
        
        my_point = Point()
        self.marker_object.pose.position = my_point

        self.marker_object.pose.orientation.x = 0
        self.marker_object.pose.orientation.y = 0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0

        self.marker_object.scale.x = 0.1
        self.marker_object.scale.y = 0.1
        self.marker_object.scale.z = 0.1
    
        self.marker_object.color.r = 0.0
        self.marker_object.color.g = 0.0
        self.marker_object.color.b = 1.0
        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 0.8
            
        # If we want it for ever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0)
    
    def update_pose(self, x, y, z, index):


        self.marker_object.id = index

        my_point = Point()

        my_point.x = x
        my_point.y = y
        my_point.z = z
        self.marker_object.pose.position = my_point


        # Random Colour
        red = random.random()
        green = random.random()
        blue = random.random()

        self.marker_object.color.r = red
        self.marker_object.color.g = green
        self.marker_object.color.b = blue

    def publish_point(self,x, y, z, index):
        """
        here Yaw will be the only one becuase it referes to he Z rotation
        """
        self.update_pose(x, y, z, index)
        self.marker_objectlisher.publish(self.marker_object)

    def start(self, x, y, z):
        rate = rospy.Rate(10)
        z_value = 0.0
        index = 0
        delta = 0.1
        up_flag = True
        z_MAX = 1.5
        z_MIN = -1.0
        theta = 0
        delta_theta = 0.2
        while not rospy.is_shutdown():
            self.publish_point(x, y, z , index=index)

            
            theta += delta_theta
            if theta >= 2*pi:
                theta = 0.0

                if up_flag:
                    z_value += delta
                else:
                    z_value -= delta

                if z_value >= z_MAX:
                    up_flag = False
                
                if z_value <= z_MIN:
                    up_flag = True

            index += 1
            print("Z = "+str(z_value))
            rate.sleep()
   



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
    rospy.init_node('marker_basic_node', anonymous=True)
    markerbasics_object = MarkerBasics()

    rospy.spin()

