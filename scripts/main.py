#!/usr/bin/env python -W ignore::DeprecationWarning
"""
Starter script for EE106B grasp planning lab
Author: Chris Correa
"""
import numpy as np
import scipy
import sys
import argparse
import time

# AutoLab imports
from autolab_core import RigidTransform
import trimesh

# 106B lab imports
from lab2.policies import GraspingPolicy
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Point 


try:
    import rospy
    import tf
    from baxter_interface import gripper as baxter_gripper
    from path_planner import PathPlanner
    ros_enabled = True
except:
    print 'Couldn\'t import ROS.  I assume you\'re running this on your laptop'
    ros_enabled = False
from lab2.utils import *
from tf import transformations

def lookup_transform(to_frame, from_frame='base'):
    """
    Returns the AR tag position in world coordinates 

    Parameters
    ----------
    to_frame : string
        examples are: ar_marker_7, gearbox, pawn, ar_marker_3, etc
    from_frame : string
        lets be real, you're probably only going to use 'base'

    Returns
    -------
    :obj:`autolab_core.RigidTransform` AR tag position or object in world coordinates
    """
    if not ros_enabled:
        print 'I am the lookup transform function!  ' \
            + 'You\'re not using ROS, so I\'m returning the Identity Matrix.'
        return RigidTransform(to_frame=from_frame, from_frame=to_frame)
    listener = tf.TransformListener()
    attempts, max_attempts, rate = 0, 10, rospy.Rate(1.0)
    while attempts < max_attempts:
        try:
            # print("in try")
            # print("1 here")
            t = listener.getLatestCommonTime(from_frame, to_frame)
            # print("here")
            tag_pos, tag_rot = listener.lookupTransform(from_frame, to_frame, t)
            real_rot = np.array([tag_rot[3], tag_rot[0], tag_rot[1], tag_rot[2]])


            
            # real_rot = np.array([tag_rot[1], tag_rot[2], tag_rot[3], tag_rot[0]])

            
            break


        except:
            rate.sleep()
            attempts += 1
    # print("t",t)
    # print("tag_pos", tag_pos)
    # # print("tag_rot", tag_rot)
    # print("real_rot", real_rot)

    ### comment out when using ar tag

    # tag_rot = [0, 0, 1, 0]
    # tag_pos = [0.570, -0.184, -.243] #changed this so it would work for right gripper better
    ### comment out when using ar tag
    rot = RigidTransform.rotation_from_quaternion(real_rot)
    # rot = tag_rot
    # print(from_frame, to_frame)
    return RigidTransform(rot, tag_pos, to_frame=from_frame, from_frame=to_frame)


def publish_grasp(grasp_object):

    publisher, listener = tf.TransformBroadcaster(), tf.TransformListener()
    publisher.sendTransform(
    grasp_object.translation,
    tf.transformations.quaternion_from_matrix(grasp_object.matrix),
    listener.getLatestCommonTime("base", "base"),
    'grasp',
    'base',
    )
       


def execute_grasp(T_grasp_world, planner, gripper):
    """
    takes in the desired hand position relative to the object, finds the desired 
    hand position in world coordinates.  Then moves the gripper from its starting 
    orientation to some distance BEHIND the object, then move to the  hand pose 
    in world coordinates, closes the gripper, then moves up.  
    
    Parameters
    ----------
    T_grasp_world : :obj:`autolab_core.RigidTransform`
        desired position of gripper relative to the world frame
    """
    def close_gripper():
        """closes the gripper"""
        gripper.close(block=True)
        rospy.sleep(1.0)

    def open_gripper():
        """opens the gripper"""
        gripper.open(block=True)
        rospy.sleep(1.0)

    print("ENTERED EXECUTE!!!!!!!!!!!!!")


    def construct_four_by_four(rotation_matrix, translation):
        """
        input:
        rotation_matrix: 3x3 np.array
        translation: 3x np.array

        output:
        4x4 np.array
        """
        print("translation ", translation)
        print("rotation ", rotation)

        three_by_four_matrix = np.concatenate((rotation, translation), axis=1)
        four_by_four = np.concatenate((three_by_four_matrix, np.array([0,0,0,1]).reshape(1,4)), axis=0)
        # four_by_four = np.concatenate((three_by_four_matrix, np.array([0,0,0,1])[:, None]), axis=1)
        return four_by_four

    def to_start_position(rotation_matrix , translation):
        """
        input:translation :3x1 np.array
              rotation:    3x3 np.array 

        Move back 10 centimeters from the object position to the start position
        """
        four_by_four = construct_four_by_four(rotation_matrix, translation)
        b = np.array([[0],
                  [0],
                  [-0.10],
                  [1]])
        start_position = np.dot(four_by_four, b)
        start_position = start_position[0:3]
        print("start_position: ", start_position)
        return start_position








    #Uncomment the following two lines when the metrics is working
    translation = T_grasp_world.translation.reshape(3,1)

    print("tranlation in T_grasp_world: ", translation)

    rotation =  T_grasp_world.rotation

    print("rotation  in T_grasp_world: ", rotation)

    #commented code block 1

    #Move to start position
    gripper.calibrate()
    calibrated_translation = translation
    calibrated_translation[1] = calibrated_translation[1]
    calibrated_translation[2] = calibrated_translation[2]+.012 
    start_position_translation = to_start_position(rotation, calibrated_translation)
    pose0 = create_pose_from_rigid_transform(construct_four_by_four(rotation, start_position_translation))
    plan0 = planner.plan_to_pose(pose0, list())
    inp = raw_input('Press <Enter> to move to start position, or \'exit\' to exit')
    open_gripper()
    planner.execute_plan(plan0)
    inp = raw_input('Motion plan worked? (y/n)')
    if inp == 'n':
        return
    time.sleep(1)


    #Move to object
    pose1 = create_pose_from_rigid_transform(construct_four_by_four(rotation, calibrated_translation))
    plan1 = planner.plan_to_pose(pose1, list())
    inp = raw_input('Press <Enter> to move to object position, or \'exit\' to exit')
    try: 
        planner.execute_plan(plan1)
    except:
        planner.execute_plan(plan1)

    inp = raw_input('Press <Enter> to close the gripper, or \'exit\' to exit')
    close_gripper()
    time.sleep(1)

    #Lift up object
    lift_up = translation
    lift_up[2] = translation[2] + 0.05
    pose2 = create_pose_from_rigid_transform(construct_four_by_four(rotation, lift_up))
    plan2 = planner.plan_to_pose(pose2, list())
    inp = raw_input('Press <Enter> to move to mid air position, or \'exit\' to exit')
    if inp == "exit":
        return

    planner.execute_plan(plan2)
    time.sleep(1)


   #Pose 3 move to the destination in mid air 
    intermediate2 = lift_up
    intermediate2[1] =  intermediate2[1] - 0.1
    pose = create_pose_from_rigid_transform(construct_four_by_four(rotation, intermediate2))
    plan4 = planner.plan_to_pose(pose, list())
    inp = raw_input('Press <Enter> to move to intermediate2 position, or \'exit\' to exit')
    if inp == "exit":
        return
    planner.execute_plan(plan4)




    #Pose 4 move to the destination
    destination = intermediate2
    destination[2] =  destination[2] - 0.05
    pose = create_pose_from_rigid_transform(construct_four_by_four(rotation, destination))
    plan5 = planner.plan_to_pose(pose, list())
    inp = raw_input('Press <Enter> to move to destination position, or \'exit\' to exit')
    if inp == "exit":
        return

    planner.execute_plan(plan5)
    open_gripper()


    """
    Note that part of the lab is also to try picking the object up and moving it to a new position and releasing it, so eventually we'll need that (see lab doc)
    """

    # raise NotImplementedError

def parse_args():
    """
    Pretty self explanatory tbh
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-obj', type=str, default='gearbox', help=
        """Which Object you\'re trying to pick up.  Options: gearbox, nozzle, pawn.  
        Default: gearbox"""
    )
    parser.add_argument('-n_vert', type=int, default=1000, help=
        'How many vertices you want to sample on the object surface.  Default: 1000'
    )
    parser.add_argument('-n_facets', type=int, default=32, help=
        """You will approximate the friction cone as a set of n_facets vectors along 
        the surface.  This way, to check if a vector is within the friction cone, all 
        you have to do is check if that vector can be represented by a POSITIVE 
        linear combination of the n_facets vectors.  Default: 32"""
    )
    parser.add_argument('-n_grasps', type=int, default=500, help=
        'How many grasps you want to sample.  Default: 500')
    parser.add_argument('-n_execute', type=int, default=5, help=
        'How many grasps you want to execute.  Default: 5')
    parser.add_argument('-metric', '-m', type=str, default='compute_force_closure', help=
        """Which grasp metric in grasp_metrics.py to use.  
        Options: compute_force_closure, compute_gravity_resistance, compute_custom_metric"""
    )
    parser.add_argument('-arm', '-a', type=str, default='left', help=
        'Options: left, right.  Default: left'
    )
    parser.add_argument('--baxter', action='store_true', help=
        """If you don\'t use this flag, you will only visualize the grasps.  This is 
        so you can run this on your laptop"""
    )
    parser.add_argument('--debug', action='store_true', help=
        'Whether or not to use a random seed'
    )
    return parser.parse_args()

if __name__ == '__main__':

    args = parse_args()
    rospy.init_node('lab2')
    if args.debug:
        np.random.seed(0)

    # Mesh loading and pre-processing
    mesh = trimesh.load_mesh('objects/{}.obj'.format(args.obj))
    T_obj_world = lookup_transform(args.obj)
    mesh.apply_transform(T_obj_world.matrix)
    mesh.fix_normals()

    # This policy takes a mesh and returns the best actions to execute on the robot
    grasping_policy = GraspingPolicy(
        args.n_vert, 
        args.n_grasps, 
        args.n_execute, 
        args.n_facets, 
        args.metric
    )
    # Each grasp is represented by T_grasp_world, a RigidTransform defining the 
    # position of the end effector
    T_grasp_worlds = grasping_policy.top_n_actions(mesh, args.obj)

    # Execute each grasp on the baxter / sawyer
    if args.baxter:
        gripper = baxter_gripper.Gripper(args.arm)
        planner = PathPlanner('{}_arm'.format(args.arm))


        for T_grasp_world in T_grasp_worlds:
            # publish_grasp(T_grasp_world)
            repeat = True
            while repeat:
                execute_grasp(T_grasp_world, planner, gripper)
                repeat = raw_input("repeat? [y|n] ") == 'y'

