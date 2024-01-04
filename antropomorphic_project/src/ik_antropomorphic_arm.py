#!/usr/bin/env python3

from math import atan2, pi, sin, cos, pow, sqrt, acos



from dataclasses import dataclass
@dataclass

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

    def compute_ik(self, end_effector_pose, elbow_configuration_2 = "plus", elbow_configuration_3 = "plus" ):
        
        # Initialization
        Pee_x = end_effector_pose.Pee_x
        Pee_y = end_effector_pose.Pee_y

        # We get all the DH parameters
        r1 = self.get_dh_param("r1")
        r2 = self.get_dh_param("r2")
        r3 = self.get_dh_param("r3")

        print("Input Data===== theta_2_config CONFIG ="+str(elbow_configuration_2))
        print("Input Data===== theta_3_config CONFIG ="+str(elbow_configuration_3))

        # We declare all the equations for theta1, theta2, theta3 and auxiliary
        #########################################################################


        ## WE HAVE TO CHECK THAT ITS POSSIBLE
        possible_solution = True


        theta_array = []

        G = 0.5*(-2 + pow(Pee_x,2) + pow(Pee_y,2) + pow(Pee_z,2))

        if elbow_configuration_2 == "plus":
            if elbow_configuration_3 == "plus":
                theta_1 = atan2(Pee_y,Pee_x)
                theta_3 = acos(G)
                theta_2 = acos(Pee_y/(2*sin(theta_1)*cos(theta_3*0.5)))-theta_3*0.5

            else :
                theta_1 = atan2(Pee_y,Pee_x)
                theta_3 = -acos(G)
                theta_2 = acos(Pee_y/(2*sin(theta_1)*cos(theta_3*0.5)))-theta_3*0.5
        else:
            if elbow_configuration_3 == "plus":
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

def calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, elbow_config_2 = "plus", elbow_config_3 = "plus"):

    ik = ComputeIk(DH_parameters = DH_parameters)
    end_effector_pose = EndEffectorWorkingSpace(Pee_x = Pee_x,
                                                Pee_y = Pee_y,
                                                Pee_z = Pee_z)

    thetas, possible_solution = ik.compute_ik(  end_effector_pose=end_effector_pose,
                                                elbow_configuration_2 = elbow_config_2,
                                                elbow_configuration_3 = elbow_config_3)

    print("("+str(thetas)+"," + str(possible_solution) + ")\n")
    return thetas, possible_solution

if __name__ == '__main__':
    

    r1 = 0.0
    r2 = 1.0
    r3 = 1.0

    # theta_i here are valriables of the joints
    # We only fill the ones we use in the equations, the others were already 
    # replaced in the Homogeneous matrix
    DH_parameters={"r1":r1,
                    "r2":r2,
                    "r3":r3}

    Pee_x = 0.5
    Pee_y = 0.6
    Pee_z = 0.7

    calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, Pee_z=Pee_z, DH_parameters=DH_parameters, elbow_config_2="plus", elbow_config_3="plus")
    calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, Pee_z=Pee_z, DH_parameters=DH_parameters, elbow_config_2="plus", elbow_config_3="minus")
    calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, Pee_z=Pee_z, DH_parameters=DH_parameters, elbow_config_2="minus", elbow_config_3="plus")
    calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, Pee_z=Pee_z, DH_parameters=DH_parameters, elbow_config_2="minus", elbow_config_3="minus")