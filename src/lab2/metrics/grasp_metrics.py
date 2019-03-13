#!/usr/bin/env python -W ignore::DeprecationWarning
"""
Grasp Metrics for EE106B grasp planning lab
Author: Chris Correa
"""
# may need more imports
import numpy as np
from lab2.utils import vec, adj, look_at_general
import cvxpy as cp
import math

def compute_force_closure(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    Compute the force closure of some object at contacts, with normal vectors 
    stored in normals You can use the line method described in HW2.  if you do you 
    will not need num_facets

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors 
        will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    # YOUR CODE HERE
    # check if vert2 in vert1 cone
    score = 0
    p1 = vertices[0]
    p2 = vertices[1]
    norm1 = normals[0]
    norm2 = normals[1]
    
    if is_in_cone(p1,p2,norm1,mu) and is_in_cone(p2,p1,norm2,mu):
        score = 1
    return score

def is_in_cone(p1,p2,norm1,mu):
    """
    Author: Ryan
    Determines if p2 is in the friction cone of p1

    Parameters
    ----------
    p1 : coordinate of point 1 in world frame
    p2 : coordinate of point 2 in world frame
    norm1 : normal direction of force 1 in world frame
    mu : coefficient of friction

    Returns
    -------
    bool : whether p2 is in the friction cone of p1
    """
    
    Gwa = look_at_general(p1,norm1)
    pt = np.array([p2[0],p2[1],p2[2],1])
    pa = np.dot(Gwa,pt)
    if math.sqrt(pa[0]**2+pa[1]**2) <= mu*pa[2]:
        return True
    else:
        return False


def get_grasp_map(vertices, normals, num_facets, mu, gamma):
    """ 
    defined in the book on page 219.  Compute the grasp map given the contact
    points and their surface normals

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors 
        will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient

    Returns
    -------
    :obj:`numpy.ndarray` grasp map
    """
    # YOUR CODE HERE 
    # Ryan code, uses utils for most of the work
    finger = np.matrix([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,0], [0,0,0,0], [0,0,0,1]])
    rot1 = look_at_general(vertices[0],normals[0])
    adj1 = adj(rot1)
    rot2 = look_at_general(vertices[1],normals[1])
    adj2 = adj(rot2)
    G1 = np.dot(adj1,finger)
    G2 = np.dot(adj2,finger)
    G = np.hstack((G1,G2))
    return G

def contact_forces_exist(vertices, normals, num_facets, mu, gamma, desired_wrench):
    """
    Compute whether the given grasp (at contacts with surface normals) can produce 
    the desired_wrench.  will be used for gravity resistance. 

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors 
        will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    desired_wrench : :obj:`numpy.ndarray`
        potential wrench to be produced

    Returns
    -------
    bool : whether contact forces can produce the desired_wrench on the object
    """
    # YOUR CODE HERE
    A = get_grasp_map(vertices, normals, num_facets, mu, gamma)
    print("desired wrench: ", desired_wrench)
    print("grasp map", A)
    b = desired_wrench
    bb = np.array([b]).T
    print('bbshape',bb.shape)
    thresh = 0.02*length(desired_wrench)
    print("threshold", thresh)



    x1 = cp.Variable()
    x2 = cp.Variable()
    x3 = cp.Variable()
    x4 = cp.Variable()
    x5 = cp.Variable()
    x6 = cp.Variable()
    x7 = cp.Variable()
    x8 = cp.Variable()
    
    objective = cp.Minimize(cp.sum_squares(A[:,0]*x1 + A[:,1]*x2 + A[:,2]*x3 + A[:,3]*x4 + A[:,4]*x5 + A[:,5]*x6 + A[:,6]*x7 + A[:,7]*x8 - bb))
    #constraints = [x[2]<=3, .1<=x[2], x[6]<=3, .1<=x[6]]
    constraints1 = [x3>=0, x7>=0, x4-gamma*x3<=0, x8-gamma*x7<=0, x4+gamma*x3>=0, x8+gamma*x7>=0, x1+x2-mu*x3<=0, x5+x6-mu*x7<=0, x1>=0, x2>=0, x5>=0, x6>=0]
    constraints2 = [x3>=0, x7>=0, x4-gamma*x3<=0, x8-gamma*x7<=0, x4+gamma*x3>=0, x8+gamma*x7>=0, x1-x2-mu*x3<=0, x5-x6-mu*x7<=0, x1>=0, x2<=0, x5>=0, x6<=0]
    constraints3 = [x3>=0, x7>=0, x4-gamma*x3<=0, x8-gamma*x7<=0, x4+gamma*x3>=0, x8+gamma*x7>=0, -x1-x2-mu*x3<=0, -x5-x6-mu*x7<=0, x1<=0, x2<=0, x5<=0, x6<=0]
    constraints4 = [x3>=0, x7>=0, x4-gamma*x3<=0, x8-gamma*x7<=0, x4+gamma*x3>=0, x8+gamma*x7>=0, -x1+x2-mu*x3<=0, -x5+x6-mu*x7<=0, x1<=0, x2>=0, x5<=0, x6>=0]

    prob1 = cp.Problem(objective, constraints1)
    prob2 = cp.Problem(objective, constraints2)
    prob3 = cp.Problem(objective, constraints3)
    prob4 = cp.Problem(objective, constraints4)

    result = prob1.solve()
    print(prob1.status)
    print(prob1.status == 'optimal')
    x_res = np.array([x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value])
    print("akdasd: ", x_res)
    if prob1.status == 'optimal':
        error_res = length(np.squeeze(np.asarray(np.dot(A,x_res) - b)))
        if error_res <= thresh:
            return True
    result = prob2.solve()
    x_res = np.array([x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value])
    print("akdasd: ", x_res)
    if prob2.status == 'optimal':
        error_res = length(np.squeeze(np.asarray(np.dot(A,x_res) - b)))
        if error_res <= thresh:
            return True
    result = prob3.solve()
    x_res = np.array([x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value])
    print("akdasd: ", x_res)
    if prob3.status == 'optimal':
        error_res = length(np.squeeze(np.asarray(np.dot(A,x_res) - b)))
        if error_res <= thresh:
            return True
    result = prob4.solve()
    x_res = np.array([x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value])
    print("akdasd: ", x_res)
    if prob4.status == 'optimal':
        error_res = length(np.squeeze(np.asarray(np.dot(A,x_res) - b)))
        if error_res <= thresh:
            return True


    # print(x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value)
    # print('status: ', prob.status, error_res)
    
    
    return False

def compute_gravity_resistance(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    Gravity produces some wrench on your object.  Computes whether the grasp can 
    produce and equal and opposite wrench

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will 
        be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    # YOUR CODE HERE (contact forces exist may be useful here)
    score = 0
    W = np.transpose(np.array([0, 0, -9.81*object_mass, 0, 0, 0]))
    if contact_forces_exist(vertices, normals, num_facets, mu, gamma, -W):
        score = 1
    return score

def compute_custom_metric(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    I suggest Ferrari Canny, but feel free to do anything other metric you find. 

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will 
        be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    # YOUR CODE HERE :)
    """
    Ferrari Canny
    I have an ok idea of how to do this
    """
    # might want to split the grasp map up into 2 halves
    # first check if in FC
    print("ENTERED CUSTOM")
    if compute_force_closure(vertices, normals, num_facets, mu, gamma, object_mass) == 0:
        return 0

    G = get_grasp_map(vertices, normals, num_facets, mu, gamma)
    fz = 1
    f_1 = np.array([mu*fz,0,fz,0])
    f_2 = np.array([0,mu*fz,fz,0])
    f_3 = np.array([-mu*fz,0,fz,0])
    f_4 = np.array([0,-mu*fz,fz,0])
    fs = np.array([f_1,f_2,f_3,f_4])
    w_max = []
    for f1 in fs:
        for f2 in fs:
            f = np.hstack((f1,f2))
            w = np.dot(G,f)
            print(w)
            w_max.append(np.linalg.norm(w))

    print("Wmax: ", w_max)
    w_min = min(w_max)
    print("Wmin: ", w_min)

    return w_min 
