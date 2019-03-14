#!/usr/bin/env python -W ignore::DeprecationWarning
"""
Grasping Policy for EE106B grasp planning lab
Author: Chris Correa
"""
import numpy as np
from math import sin, cos
import time
import tf

# Autolab imports
from autolab_core import RigidTransform
import trimesh
from visualization import Visualizer3D as vis3d

# 106B lab imports
from lab2.metrics import (
    compute_force_closure, 
    compute_gravity_resistance, 
    compute_custom_metric
)
from lab2.utils import length, normalize, look_at_general, create_pose_from_rigid_transform

# YOUR CODE HERE
# probably don't need to change these (BUT confirm that they're correct)
MAX_HAND_DISTANCE = .07     #used to be .04, but obviously .06 works better
MIN_HAND_DISTANCE = .031     #used to be .01
CONTACT_MU = 0.5
CONTACT_GAMMA = 0.1
GRIPPER_LENGTH = 0.1 # how long the gripper is

# TODO
OBJECT_MASS = {'gearbox': .25, 'nozzle': .25, 'pawn': .25}


class GraspingPolicy():
    def __init__(self, n_vert, n_grasps, n_execute, n_facets, metric_name):
        """
        Parameters
        ----------
        n_vert : int
            We are sampling vertices on the surface of the object, and will use pairs of 
            these vertices as grasp candidates
        n_grasps : int
            how many grasps to sample.  Each grasp is a pair of vertices
        n_execute : int
            how many grasps to return in policy.action()
        n_facets : int
            how many facets should be used to approximate the friction cone between the 
            finger and the object
        metric_name : string
            name of one of the function in src/lab2/metrics/metrics.py
        """
        self.n_vert = n_vert
        self.n_grasps = n_grasps
        self.n_execute = n_execute
        self.n_facets = n_facets
        # This is a function, one of the functions in src/lab2/metrics/metrics.py
        self.metric = eval(metric_name)


    def vertices_to_baxter_hand_pose(self, grasp_vertices, approach_direction, Rot, obj_name = 'gearbox'):
        """
        takes the contacts positions in the object frame and returns the hand pose T_obj_gripper
        BE CAREFUL ABOUT THE FROM FRAME AND TO FRAME.  the RigidTransform class' frames are 
        weird.
        
        Parameters
        ----------
        grasp_vertices : 2x3 :obj:`numpy.ndarray`
            position of the fingers in object frame
        approach_direction : 3x' :obj:`numpy.ndarray`
            there are multiple grasps that go through contact1 and contact2.  This describes which 
            orientation the hand should be in

        Returns
        -------
        :obj:`autolab_core:RigidTransform` Hand pose in the object frame Note: in object frame
        """
        # YOUR CODE HERE 




        v1, v2 = grasp_vertices
        center = (v1 + v2) / 2
        #print("v1,v2,center", v1, v2, center)
        #print("App_dir:", approach_direction)
        rot = look_at_general(center, approach_direction)
        wrist_position = center #- approach_direction*GRIPPER_LENGTH # note that this only works b/c I made sure approach_direction is normalized
        # print("wrist_position ", wrist_position)
        print("approach direction: ", approach_direction)
        print("vertices", v1, v2)
        # time.sleep(10)

        # wrist_position assumes is the position of the wrist (duh) and not the grippers

        #print("rotation", rot)
        # rot = rot[:3, :3]
        modified_identity = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
        rot = np.dot(modified_identity, np.asarray(Rot[:3, :3]))
        rpy = tf.transformations.euler_from_matrix(rot)
        print("!!!!!!!!!!RPY!!!!!!!!!!")
        print("rot in rpy", rpy)
        g =  RigidTransform(rot, wrist_position, from_frame='grasp', to_frame=obj_name)
        print(" ## g ##### ", g)
        # I quite not sure here
        return RigidTransform(rot, wrist_position, from_frame='grasp', to_frame=obj_name)

        # v1 = look_at_general(v1, approach_direction)
        # v2 = look_at_general(v2, approach_direction)

        #raise NotImplementedError

    def sample_grasps(self, vertices, normals):
        """
        Samples a bunch of candidate grasps.  You should randomly choose pairs of vertices and throw out
        pairs which are too big for the gripper, or too close too the table.  You should throw out vertices 
        which are lower than ~3cm of the table.  You may want to change this.  Returns the pairs of 
        grasp vertices and grasp normals (the normals at the grasp vertices)

        Parameters
        ----------
        vertices : nx3 :obj:`numpy.ndarray`
            mesh vertices
        normals : nx3 :obj:`numpy.ndarray`
            mesh normals
        T_ar_object : :obj:`autolab_core.RigidTransform`
            transform from the AR tag on the paper to the object

        Returns
        -------
        n_graspsx2x3 :obj:`numpy.ndarray`
            grasps vertices.  Each grasp containts two contact points.  Each contact point
            is a 3 dimensional vector and there are n_grasps of them, hence the shape n_graspsx2x3
        n_graspsx2x3 :obj:`numpy.ndarray`
            grasps normals.  Each grasp containts two contact points.  Each vertex normal
            is a 3 dimensional vector, and there are n_grasps of them, hence the shape n_graspsx2x3
        """
        grasp_vertices = []
        grasp_normals = []
        table_pos = -0.243   # here it is the position of table in z direction, -0.243 is Aryton's and -0.198 is Archytas'
        l = len(vertices)
        num = 0

        print "sample_grasps"

        while num < self.n_grasps:
            # randomly generate two vertices and check whether  
            index = np.random.randint(0, l-1, size=2)
            v1, v2 = vertices[index[0]], vertices[index[1]]
            n1, n2 = normals[index[0]], normals[index[1]]
            # print("index", index, v1, v2)
            dis = length(v1 - v2)
            cos = np.dot(n1, n2)/length(n1)/length(n2)
            #normal is 3 for some reason. maybe we pass in the wrong vectors in some other function?
            # print("normals shape: " + str(normals.shape))
            # print('dis',dis,'cos',cos)

            #!!!!!!!careful about the height constraints
            if dis <= MAX_HAND_DISTANCE and dis >= MIN_HAND_DISTANCE and (v1[2] - table_pos >= 0.03) and (v2[2] - table_pos >= 0.03) and (cos < 0) and (abs(v1[2] - v2[2]) <= 0.03):
                num += 1
                v = np.vstack([v1, v2])
                n = np.vstack([n1, n2])
                grasp_vertices.append(v)
                grasp_normals.append(n) 

        return (np.array(grasp_vertices), np.array(grasp_normals))   


        # raise NotImplementedError

    def score_grasps(self, grasp_vertices, grasp_normals, object_mass):
        """
        takes mesh and returns pairs of contacts and the quality of grasp between the contacts, sorted by quality
        
        Parameters
        ----------
        grasp_vertices : n_graspsx2x3 :obj:`numpy.ndarray`
            grasps.  Each grasp containts two contact points.  Each contact point
            is a 3 dimensional vector, and there are n_grasps of them, hence the shape n_graspsx2x3
        grasp_normals : mx2x3 :obj:`numpy.ndarray`
            grasps normals.  Each grasp containts two contact points.  Each vertex normal
            is a 3 dimensional vector, and there are n_grasps of them, hence the shape n_graspsx2x3

        Returns
        -------
        :obj:`list` of int
            grasp quality for each 
        """
        quality = []
        for i in range(self.n_grasps):
            quality.append(self.metric(grasp_vertices[i], grasp_normals[i], self.n_facets, CONTACT_MU, CONTACT_GAMMA, object_mass))
        return quality

        # raise NotImplementedError

    def vis(self, mesh, grasp_vertices, grasp_qualities):
        """
        Pass in any grasp and its associated grasp quality.  this function will plot
        each grasp on the object and plot the grasps as a bar between the points, with
        colored dots on the line endpoints representing the grasp quality associated
        with each grasp
        
        Parameters
        ----------
        mesh : :obj:`Trimesh`
        grasp_vertices : mx2x3 :obj:`numpy.ndarray`
            m grasps.  Each grasp containts two contact points.  Each contact point
            is a 3 dimensional vector, hence the shape mx2x3
        grasp_qualities : mx' :obj:`numpy.ndarray`
            vector of grasp qualities for each grasp
        """
        vis3d.mesh(mesh)

        dirs = normalize(grasp_vertices[:,0] - grasp_vertices[:,1], axis=1)
        midpoints = (grasp_vertices[:,0] + grasp_vertices[:,1]) / 2
        grasp_endpoints = np.zeros(grasp_vertices.shape)
        grasp_vertices[:,0] = midpoints + dirs*MAX_HAND_DISTANCE/2
        grasp_vertices[:,1] = midpoints - dirs*MAX_HAND_DISTANCE/2

        for grasp, quality in zip(grasp_vertices, grasp_qualities):
            color = [min(1, 2*(1-quality)), min(1, 2*quality), 0, 1]
            if color != [1,0,0,1]:
                good_grasp += 1
                # vis3d.plot3d(grasp, color=color, tube_radius=.0005)
            vis3d.plot3d(grasp, color=color, tube_radius=.0005)
        vis3d.show()

    def top_n_actions(self, mesh, obj_name, vis=True):
        """
        Takes in a mesh, samples a bunch of grasps on the mesh, evaluates them using the 
        metric given in the constructor, and returns the best grasps for the mesh.  SHOULD
        RETURN GRASPS IN ORDER OF THEIR GRASP QUALITY.

        You should try to use mesh.mass to get the mass of the object.  You should check the 
        output of this, because from this
        https://github.com/BerkeleyAutomation/trimesh/blob/master/trimesh/base.py#L2203
        it would appear that the mass is approximated using the volume of the object.  If it
        is not returning reasonable results, you can manually weight the objects and store 
        them in the dictionary at the top of the file.

        Parameters
        ----------
        mesh : :obj:`Trimesh`
        vis : bool
            Whether or not to visualize the top grasps

        Returns
        -------
        :obj:`list` of :obj:`autolab_core.RigidTransform`
            the matrices T_grasp_world, which represents the hand poses of the baxter / sawyer
            which would result in the fingers being placed at the vertices of the best grasps
        """
        # Some objects have vertices in odd places, so you should sample evenly across 
        # the mesh to get nicer candidate grasp points using trimesh.sample.sample_surface_even()

        new_vertices, face_index = trimesh.sample.sample_surface_even(mesh, self.n_vert)
        new_normals = []
        vertices = mesh.vertices
        normals = mesh.face_normals

        object_mass = OBJECT_MASS.get(obj_name) # return desired value from OBJECT_MASS dictionary
        # assume the approach direction is vertical, may need to be change based on the real situation
        # I think we can create different approach direction for each object

        # get the corresponding normals of those candidate points 
        for i in face_index:            
            new_normals.append(normals[i])
            # print("index", i," normal",normals[i])
        new_normals = np.array(new_normals)
        (grasp_vertices, grasp_normals) = self.sample_grasps(new_vertices, new_normals)
        grasp_qualities = self.score_grasps(grasp_vertices, grasp_normals, object_mass)

        self.vis(mesh, grasp_vertices, grasp_qualities)

        best = []

        # sort the scores and return the original indexes of the scores 
        sorted_index = sorted(range(len(grasp_qualities)), key=lambda k: grasp_qualities[k], reverse = True)
        for i in range(self.n_execute):
            index = sorted_index[i]
            v = grasp_vertices[index]
            n = grasp_normals[index]

            """
            Here is my code for determining the approach direction for each grasp.
            I tested the calculations with a few examples, but it's possible it doesn't
            work perfectly. Basically we assume we start with the gripper pointed downwards
            (approach_direction=[0,0,-1]). We use geometry to calculate the Euler angles
            and then find the new approach direction by using rotation matrices Rz(theta) & Rx(theta)
            from 106A. The final approach_direction should give the new direction of the gripper 
            when oriented to make contact at the two vertices.
            """
            if v[0][1] <= v[1][1]:
                v1 = v[0]
                v2 = v[1]
            else:
                v1 = v[1]
                v2 = v[0]
            delx = v1[0] - v2[0]
            dely = v2[1] - v1[1]
            t_z = -np.arctan(delx/dely) # pi + this value or -pi - this value

            delz = v1[2] - v2[2]
            t_x = -np.arctan(delz/dely) # pi + this value or -pi - this value

            ex = np.matrix([[1,0,0],[0,cos(t_x),-sin(t_x)],[0,sin(t_x),cos(t_x)]])
            ez = np.matrix([[cos(t_z),-sin(t_z),0],[sin(t_z),cos(t_z),0],[0,0,1]])
            Rot = np.dot(ez,ex)

            approach_direction = np.dot(ez,np.dot(ex,np.array([0,0,-1])).T)
            approach_direction = np.squeeze(np.asarray(approach_direction))
            approach_direction = np.array([approach_direction[0],approach_direction[1],approach_direction[2]]) #I negate the middle turn b/c it works that way in my examples, I likely made a math error that made this necessary

            best.append(self.vertices_to_baxter_hand_pose(v, approach_direction, Rot, obj_name))
            




        return best