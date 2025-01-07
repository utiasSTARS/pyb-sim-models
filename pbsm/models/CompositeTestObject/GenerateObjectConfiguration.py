#!/usr/bin/env python3

from urdfpy import URDF, Inertial, rpy_to_matrix
from pathlib import Path
import numpy as np
import math
import argparse
import os

class CompositeTestObject:

    #Returns the skew-symmetric (or anti-symmetric) matrix of a 3D vector
    #such that skew(a)*b == a x b
    def skew(self, r):
        return np.array([
            [0, -r[2], r[1]],
            [r[2], 0, -r[0]],
            [-r[1], r[0], 0],
        ])

    #Compute the inertia tensor wrt frame S from the inertia tensor wrt center of mass.
    # I_com: Inertia tensor computed at the location of the center of mass about axes aligned with the object.
    # com_s: Position of the center of mass of the object with respect frame S.
    # C_cs: Orientation of the object with respect to frame S.
    # mass: Mass of the object.
    def inertiaWrtS(self, I_com, com_s, C_cs, mass):
        #1. Uses a similarity transform to align the axes of the I_com to frame S. (Rotation)
        I_cs = C_cs @ I_com @ C_cs.T
        #2. Uses the parallel axis theorem to express the inertia tensor wrt frame S. (Translation)
        I_s = I_cs - mass * self.skew(com_s) @ self.skew(com_s)
        return I_s

    #Return the mass and inertia tensor of a cuboid with given centre of mass
    #The inertia tensor is expressed wrt the origin of the com_pose
    def cuboid_inertia(self, dimensions, mass_density, com_pose):
        dx = dimensions[0]
        dy = dimensions[1]
        dz = dimensions[2]

        com_ori = com_pose[0:3,0:3]
        com_pos = com_pose[0:3,3]

        mass = dx*dy*dz*mass_density

        inertia_c = (mass/12)*np.array([[dy**2+dz**2,0,0],[0,dx**2+dz**2,0],[0,0,dx**2+dy**2]])
        inertia_o = self.inertiaWrtS(inertia_c, com_pos, com_ori, mass)

        return (mass, com_pos, inertia_o)

    #Return the mass and inertia tensor of a cylinder with given centre of mass
    #The inertia tensor is expressed wrt the origin of the com_pose
    def cylinder_inertia(self, dimensions, mass_density, com_pose):
        radius = dimensions[0]
        height = dimensions[1]

        com_ori = com_pose[0:3,0:3]
        com_pos = com_pose[0:3,3]

        mass = math.pi*radius**2 * height * mass_density

        inertia_c = (mass/12)*np.array([[3*radius**2+height**2,0,0],[0,3*radius**2+height**2,0],[0,0,6*radius**2]])
        inertia_o = self.inertiaWrtS(inertia_c, com_pos, com_ori, mass)

        return (mass, com_pos, inertia_o)

    #Return the mass and inertia tensor of a half-cylinder with given centre of the full cylinder (located in the centre of the rectangular side)
    #The Y axis of the half-cylinder goes from the centroid through the rectangular side
    #The Z axis follows the height of the half-cylinder
    #The inertia tensor is expressed wrt the origin of the com_pose
    #https://www.efunda.com/math/solids/solids_display.cfm?SolidName=HalfCircularCylinder
    def half_cylinder_inertia(self, dimensions, mass_density, centre):
        radius = dimensions[0]
        height = dimensions[1]

        com_ori = centre[0:3,0:3]
        sign = (centre[1,3]/abs(centre[1,3]))
        com_pos = centre[0:3,3] + [0,sign*(4*radius)/(3*math.pi),0]

        mass = math.pi*radius**2 * height * mass_density / 2

        inertia_c = mass*np.array([[(1/4)*radius**2+(1/3)*height**2,0,0],[0,(1/4)*radius**2+(1/3)*height**2,0],[0,0,(1/2)*radius**2]])
        inertia_o = self.inertiaWrtS(inertia_c, centre[0:3,3], com_ori, mass)

        return (mass, com_pos, inertia_o)


    #Returns a 4x4 pose matrix from roll-pitch-yaw (r,p,y) and translation t=(tx,ty,tz)
    def com_pose(self, r,p,y, t):
        tx, ty, tz = t[0:3]
        ori = rpy_to_matrix([r,p,y])
        pos = np.array([tx,ty,tz])
        pose = np.array([[ori[0,0],ori[0,1],ori[0,2],pos[0]],[ori[1,0],ori[1,1],ori[1,2],pos[1]],[ori[2,0],ori[2,1],ori[2,2],pos[2]],[0,0,0,1]])
        return pose

    def __init__(self, bolts_attached, steel_weights_added, plastic_weights_added, OUTPUT_URDF, print_params=False, linkxacro=False):
        #Input URDF relative path
        OBJECT_URDF_PATH = os.path.dirname(__file__) +"/urdf/compositeH.urdf"

        #If a XACRO is requested, verify that the extension of the output file is correct
        if linkxacro :
            OUTPUT_URDF = str(Path(OUTPUT_URDF).with_suffix('.xacro'))

        #Replace 'package://' with relative paths.
        #urdfpy does not support 'package://' but only relative paths
        with open(OBJECT_URDF_PATH, "rt") as fin:
            with open(OUTPUT_URDF, "wt") as fout:
                for line in fin:
                    fout.write(line.replace('package://composite-test-object/', ''))

        #Load the modified URDF
        obj = URDF.load(OUTPUT_URDF)
        body = obj.links[0]

        #15 bolting through-all holes can be either filled with air or with steel
        #10 weight holes can be filled with air, plastic or steel

        #1000 Kg/m^3 = 1 g/cm^3
        #Simulation values: PLA=260, ABS=1040, Steel=7770
        #Real-world values: PLA=491, ABS=1321, Steel=7847
        PLA_struct_density  = 491  #Assumes a 20% infill
        ABS_density         = 1321
        steel_density       = 7847  
        bolt_density        = 6730

        #Initially, we assume that the whole body is filled with plastic.
        mm_to_m = 1e-3

        #Weight holes specifications
        #There is a clearance in the 3D-printed version
        #Simulation: Radius=12.7
        #Real-world: Radius=15
        wh_r = mm_to_m*15 #Radius
        wh_h = mm_to_m*25.4 #Height

        #Weight specifications
        w_r = mm_to_m*12.7 #Radius
        w_h = mm_to_m*25.4 #Height

        #Bolt holes specifications
        b_r = mm_to_m*3.3 #Radius
        b_h = mm_to_m*50  #Height

        #Position of the centroid of the weight holes and bolt holes relative to origin
        weight_pos = mm_to_m*np.array([[-65,125,25],[0,125,25],[65,125,25],[0,75,25],[0,25,25],[0,-25,25],[0,-75,25],[-65,-125,25],[0,-125,25],[65,-125,25]])
        bolt_pos   = mm_to_m*np.array([[0,150,25],[-90,125,25],[-40,125,25],[40,125,25],[90,125,25],[0,100,25],[0,50,25],[0,0,25],[0,-50,25],[0,-100,25],[-90,-125,25],[-40,-125,25],[40,-125,25],[90,-125,25],(0,-150,25)])

        #Shorthands
        com_pose                = self.com_pose
        cuboid_inertia          = self.cuboid_inertia
        half_cylinder_inertia   = self.half_cylinder_inertia
        cylinder_inertia        = self.cylinder_inertia


        primitives = [  #Add base plastic cuboids
                        cuboid_inertia(mm_to_m*np.array([50,200,50]), PLA_struct_density, com_pose(0,0,0, mm_to_m*np.array([0,0,25]))),
                        cuboid_inertia(mm_to_m*np.array([200,50,50]), PLA_struct_density, com_pose(0,0,0, mm_to_m*np.array([0,125,25]))),
                        cuboid_inertia(mm_to_m*np.array([200,50,50]), PLA_struct_density, com_pose(0,0,0, mm_to_m*np.array([0,-125,25]))),
                        
                        #Add base plastic bolt supports
                        half_cylinder_inertia(mm_to_m*np.array([7.5,50]), PLA_struct_density, com_pose(0,0,0, bolt_pos[0])),
                        half_cylinder_inertia(mm_to_m*np.array([7.5,50]), PLA_struct_density, com_pose(0,0,0, bolt_pos[14])),

                        #Remove plastic (negative density) for weight holes
                        cylinder_inertia([wh_r,wh_h], -PLA_struct_density, com_pose(0,0,0, weight_pos[0])),
                        cylinder_inertia([wh_r,wh_h], -PLA_struct_density, com_pose(0,0,0, weight_pos[1])),
                        cylinder_inertia([wh_r,wh_h], -PLA_struct_density, com_pose(0,0,0, weight_pos[2])),
                        cylinder_inertia([wh_r,wh_h], -PLA_struct_density, com_pose(0,0,0, weight_pos[3])),
                        cylinder_inertia([wh_r,wh_h], -PLA_struct_density, com_pose(0,0,0, weight_pos[4])),
                        cylinder_inertia([wh_r,wh_h], -PLA_struct_density, com_pose(0,0,0, weight_pos[5])),
                        cylinder_inertia([wh_r,wh_h], -PLA_struct_density, com_pose(0,0,0, weight_pos[6])),
                        cylinder_inertia([wh_r,wh_h], -PLA_struct_density, com_pose(0,0,0, weight_pos[7])),
                        cylinder_inertia([wh_r,wh_h], -PLA_struct_density, com_pose(0,0,0, weight_pos[8])),
                        cylinder_inertia([wh_r,wh_h], -PLA_struct_density, com_pose(0,0,0, weight_pos[9])),

                        #Remove plastic (negative density) for bolt holes
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[0])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[1])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[2])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[3])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[4])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[5])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[6])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[7])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[8])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[9])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[10])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[11])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[12])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[13])),
                        cylinder_inertia([b_r,b_h], -PLA_struct_density, com_pose(0,0,0, bolt_pos[14])),
                        ]

        #Add bolts
        for bolt in bolts_attached:
            i = cylinder_inertia([b_r,b_h], bolt_density, com_pose(0,0,0, bolt_pos[bolt]))
            primitives.append(i)

        #Add steel weights
        for weight in steel_weights_added:
            i = cylinder_inertia([w_r,w_h], steel_density, com_pose(0,0,0, weight_pos[weight]))
            primitives.append(i)

        #Add plastic weights
        for weight in plastic_weights_added:
            i = cylinder_inertia([w_r,w_h], ABS_density, com_pose(0,0,0, weight_pos[weight]))
            primitives.append(i)

        #Compute the global inertial parameters
        total_mass      = 0.
        total_com       = np.array([0.,0.,0.])
        total_inertia   = np.zeros((3,3))
        for part in primitives:
            total_mass      += part[0]
            total_com       += part[0]*part[1]
            total_inertia   += part[2]
        total_com = total_com / total_mass

        #Print inertial parameters
        if print_params:
            #Do not use scientific notation -- easier to copy into YAML files that way.
            np.set_printoptions(suppress=True, formatter={'float_kind':'{:10.10f}'.format})
            print(OUTPUT_URDF)
            print(total_mass)
            print(total_com)
            print(total_inertia)
            #Put back default printing options
            np.set_printoptions(edgeitems=3, infstr='inf',linewidth=75, nanstr='nan', precision=8,suppress=False, threshold=1000, formatter=None)

        #Save the inertial parameters in the URDF
        obj_mass    = total_mass                        #The mass of the link in kilograms.
        obj_inertia = total_inertia                     #The 3x3 symmetric rotational inertia matrix with respect to obj_origin_pose
        obj_origin_pose = com_pose(0,0,0, total_com)    #The pose of the center of mass relative to the link's reference frame
        body.inertial = Inertial(obj_mass, obj_inertia, obj_origin_pose)
        obj.save(OUTPUT_URDF)

        #Transforms the URDF into the link's XACRO by removing the
        #two first lines and the last line.
        if linkxacro:
            f = open(OUTPUT_URDF, 'r')
            lines = f.readlines()
            del lines[1]
            del lines[0]
            del lines[len(lines)-1]
            with open(OUTPUT_URDF, 'w') as f:
                f.writelines(lines)

def init_argparse():
        parser = argparse.ArgumentParser(description="Generate the URDF of a composite object with predictable inertial parameters.")

        parser.add_argument('-b','--bolts', metavar='POSITION', dest='bolts_pos', type=int, nargs='*',help='List of bolt locations.')
        parser.add_argument('-s','--steelweights', metavar='POSITION', dest='steel_weights_pos', type=int, nargs='*',help='List of weights locations where steel weights are placed.')
        parser.add_argument('-p','--plasticweights', metavar='POSITION', dest='plastic_weights_pos', type=int, nargs='*',help='List of weights locations where plastic weights are placed.')
        parser.add_argument('-o','--output', metavar='FILE', dest='output_urdf', nargs='?', default="GeneratedConfiguration.urdf",help='Optional filename of the output URDF to generate.')
        parser.add_argument("--print", help="Prints the resulting inertial parameters.", action="store_true")
        parser.add_argument("--linkxacro", help="Generates a XACRO file, that can be included by a parent XACRO, containing only the link.", action="store_true")
        parser.add_argument("-v", "--version", action="version",version = f"{parser.prog} version 1.0.0")

        return parser

def parse_arguments():
    parser  = init_argparse()
    args    = parser.parse_args()

    #See BoltsWeightsNumbering.pdf for Bolts and weights numbering
    if args.bolts_pos is not None:
        #Bolts numbers that are attached
        bolts_attached = args.bolts_pos
    else:
        bolts_attached = []

    if args.steel_weights_pos is not None:
        #Weights holes numbers where steel weights are added
        steel_weights_added = args.steel_weights_pos
    else:
        steel_weights_added = []

    if args.plastic_weights_pos is not None:
        #Weights holes numbers where plastic weights are added
        plastic_weights_added = args.plastic_weights_pos
    else:
        plastic_weights_added = []

    return bolts_attached, steel_weights_added, plastic_weights_added, args.output_urdf, args.print, args.linkxacro

if __name__ == '__main__':
    bolts_attached, steel_weights_added, plastic_weights_added, OUTPUT_URDF, print_params, linkxacro = parse_arguments()
    CompositeTestObject(bolts_attached, steel_weights_added, plastic_weights_added, OUTPUT_URDF, print_params, linkxacro)