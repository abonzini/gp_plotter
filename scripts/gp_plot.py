#!/usr/bin/env python3

import argparse
import sys
import rospy
import tf
import numpy as np
from std_msgs.msg import String, Empty
import math

from gp_plotter.srv import SetPrior, SetPriorResponse, SetHyp, SetHypResponse, AddPoint, AddPointResponse
from gp_plotter.GPutil import GPX, GP, TSPCov3D, SECov

from skimage import measure

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def handle_set_prior(req):
    global the_GP
    global prior_ready
    if prior_ready:
        print("ERROR! Can't define prior multiple times...")
        raise Exception
    prior_ready= True
    global new_data 
    new_data = True
    
    resolution = 40
    global d
    global h
    global center 
    center = np.array([req.x,req.y,req.z])
    bound_range_x = np.linspace(-d/2+req.x,d/2+req.x,resolution)
    bound_range_y = np.linspace(-d/2+req.y,d/2+req.y,resolution)
    bound_range_z = np.linspace(req.z, h+req.z,resolution)
    global spacing
    spacing = [np.diff(bound_range_x)[0],np.diff(bound_range_y)[0],np.diff(bound_range_z)[0]]
    xx,xy,xz = np.meshgrid(bound_range_x, bound_range_y, bound_range_z, indexing = 'ij')
    global mesh_shape 
    mesh_shape = xx.shape
    Xx = np.hstack((xx.reshape(-1,1),xy.reshape(-1,1),xz.reshape(-1,1)))
    
    the_GP.AddXx(Xx)
    the_GP.prior_params = (req.radius, req.radius, req.radius, req.radius, center)
    print("Prior changed to radius:", the_GP.prior_params[0], "And X0:", the_GP.prior_params[4])
    
    return SetPriorResponse(True)

def handle_set_hyp(req):
    global the_GP
     
    global hyp_ready 
    hyp_ready = True
    global new_data 
    new_data = True
    print("Hyp changed")
    return SetHypResponse(True)

def add_point(req):
    global next_x
    global next_y
    new_X = np.array([req.x,req.y,req.z]).reshape((1,-1))
    next_x = np.vstack((next_x, new_X))
    global the_GP
    new_Y = -the_GP.GetPriorCalc()[0](new_X)
    next_y = np.vstack((next_y, new_Y))
    global new_data 
    new_data = True
    print("Point", new_X, "added to queue with a prior value of", new_Y)
    global prior_ready
    global hyp_ready
    if prior_ready and hyp_ready:
        return AddPointResponse(True)
    return AddPointResponse(False)

def clone_mesh(req):
    global clone_reference_frame
    clone_reference_frame = req.data

def reset_request(req):
    global reset_requested
    reset_requested = True

def cov_to_rgba(covs, min_cov = 0, max_cov = 1, log = True, alpha = 1):
    rgba = []
    if log:
        min_cov = math.log(min_cov)
        max_cov = math.log(max_cov)
        covs = np.log(covs)
    for cov in covs:
        color = ColorRGBA()
        tf_cov = max(min(cov[0], max_cov), min_cov)
        tf_cov /= max_cov
        color.g = 0
        color.a = alpha
        color.r = tf_cov
        color.b = 1-tf_cov
        rgba += [color]
    return rgba

def ResetEverything():
    global the_GP, new_data, prior_ready, hyp_ready, mesh_shape, d, h, center, spacing, ax, next_x, next_y, clone_reference_frame, reset_requested
    reset_requested = False
    the_GP = GP()
    new_data = False
    prior_ready = False
    hyp_ready = False
    mesh_shape = None
    d = None
    h = None
    center = None
    spacing = None
    clone_reference_frame = None
    ax = None
    next_x = np.empty((0,3))
    next_y = np.empty((0,1))

def main(world):
    global the_GP, new_data, prior_ready, hyp_ready, mesh_shape, d, h, center, spacing, ax, clone_reference_frame, reset_requested

    mesh_pub = rospy.Publisher('mesh_publish', Marker, queue_size=10)
    mesh = Marker()
    mesh.color.r = 0
    mesh.color.g = 1
    mesh.color.b = 0
    mesh.color.a = 1
    mesh.scale.x = 1
    mesh.scale.y = 1
    mesh.scale.z = 1
    mesh.pose.orientation.x = 0.0
    mesh.pose.orientation.y = 0.0
    mesh.pose.orientation.z = 0.0
    mesh.pose.orientation.w = 1.0
    mesh.type = 11 # triangle list
    print("Created mesh publisher")
    clone_pub = rospy.Publisher('clone_mesh_publish', Marker, queue_size=10)
    print("Created clone publisher")
    
    # d and h specify the max size of the plotting box (30 cm to each side, d is diameter (side) and h is height)
    d = 0.6
    h = 0.6
    # Noise hyp defines noise of points. If low noise can fit points better and get sharp edges but may also break reconstruction
    # High noise fits curve always but always rounded
    the_GP.noise = 0.05 # Default noise (can be overwritten)
    # TSP model has no hyperparameter, only maximum object size ([0.3] default means max size of 30cm)
    the_GP.model = TSPCov3D
    the_GP.hyp = [0.30] # max size for tsp
    # TO TRY SQUARE EXPONENTIAL MODEL UNCOMMENT LINES
    # Hyperparameters: default is 0.1, larger values means a rounder shape, lower value fits better but also points are less connected overall
    #the_GP.model = SECov
    #the_GP.hyp = [1,0.1] # SE has one hyp (the other one shoudl stay at 0), 
    hyp_ready=True
    something = False
    while not rospy.is_shutdown() and not reset_requested:
        if new_data and prior_ready and hyp_ready:
            global next_x
            global next_y
            if len(next_x)>0:
                n = min(len(next_x),len(next_y)) # I check immediately the buffer
                print("Buffer of new input/output is not empty! Inputs:", next_x,"Outputs:",next_y.ravel())
                the_GP.AddX(next_x[:n, :])
                the_GP.Y = np.vstack((the_GP.Y, next_y[:n, :]))
                print("Points added, now GP has these points:", the_GP._X.Values)
                print("And these values:", the_GP.Y.ravel(), "Y size:", len(the_GP.Y))
                next_x = np.empty((0,3))
                next_y = np.empty((0,1))
                print("K size:", the_GP.K.shape)
                print("Kx size:", the_GP.Kx.shape)
            mu = np.copy(the_GP.mu)
            mu = mu.reshape(mesh_shape)
            
            verts, faces, _, _ = measure.marching_cubes(mu,0)
            verts *= spacing
            verts -= np.array([d/2,d/2,0])
            verts += center
            
            vertGP = the_GP.GetCopy()
            vertGP.AddXx(verts)
            vertCov = np.diag(vertGP.cov).reshape((-1,1))

            ptarray = []
            for triangle in verts[faces].reshape(-1,3):
                p = Point()
                p.x = triangle[0]
                p.y = triangle[1]
                p.z = triangle[2]
                ptarray += [p]
            mesh.points = ptarray
            
            face_uncertainty = vertCov[faces] # Now each polygon contains 3 bordering uncertainties
            face_uncertainty = np.amax(face_uncertainty, axis = 1)
            mesh.colors = cov_to_rgba(face_uncertainty, min_cov = the_GP.noise**2, max_cov= 0.5**2, log = False)
            
            new_data = False
            something = True
            print("New iteration plotted!")
        if something:
            mesh.header.stamp = rospy.Time.now()
            mesh.header.frame_id = world
            mesh_pub.publish(mesh)
            if clone_reference_frame is not None:
                mesh.header.stamp = rospy.Time.now()
                mesh.header.frame_id = clone_reference_frame
                clone_pub.publish(mesh)

        rospy.sleep(0.05)

if __name__ == "__main__":
    myargv = rospy.myargv(argv=sys.argv)
    if len(myargv) > 1:
        world = myargv[1]
    else:
        print('WARNING: The mesh has it\'s default world origin into the tf parent called \"world\"')
        print('Call the code gp_plot.py REFERENCE_TF to set another parent reference')
        world = 'world'
    # Set ros stuff
    rospy.init_node("GPPlotter")

    serv_prior = rospy.Service('set_prior', SetPrior, handle_set_prior)
    serv_hyp = rospy.Service('set_hyp', SetHyp, handle_set_hyp)
    serv_point = rospy.Service('add_point_gp_service', AddPoint, add_point)
    serv_clone_mesh = rospy.Service('clone_mesh', String, clone_mesh)
    reset_service = rospy.Service('reset_GP', Empty, reset_request)
    print("Now hosting services")
    #Start experiment
    while not rospy.is_shutdown():
        ResetEverything()
        main(world)
