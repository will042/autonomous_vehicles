#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *
import numpy as np
from probabilistic_lib.functions import angle_wrap, comp, compInv
from scipy.linalg import block_diag
import rospy


#============================================================================
class EKF_SLAM(object):
    '''
    Class to hold the whole EKF-SLAM.
    '''
    
    #========================================================================
    def __init__(self, x0,y0,theta0, odom_lin_sigma, 
                 odom_ang_sigma, meas_rng_noise, meas_ang_noise):
        '''
        Initializes the ekf filter
        room_map : an array of lines in the form [x1 y1 x2 y2]
        num      : number of particles to use
        odom_lin_sigma: odometry linear noise
        odom_ang_sigma: odometry angular noise
        meas_rng_noise: measurement linear noise
        meas_ang_noise: measurement angular noise
        '''
        
        # Copy parameters
        self.odom_lin_sigma = odom_lin_sigma
        self.odom_ang_sigma = odom_ang_sigma
        self.meas_rng_noise = meas_rng_noise
        self.meas_ang_noise = meas_ang_noise
        self.chi_thres = 0.103 # TODO chose your own value
       
        # Odometry uncertainty 
        self.Qk = np.array([[ self.odom_lin_sigma**2, 0, 0],\
                            [ 0, self.odom_lin_sigma**2, 0 ],\
                            [ 0, 0, self.odom_ang_sigma**2]])
        
        # Measurements uncertainty
        self.Rk=np.eye(2)
        self.Rk[0,0] = self.meas_rng_noise
        self.Rk[1,1] = self.meas_ang_noise
        
        # State vector initialization
        self.xk = np.array([x0,y0,theta0]) # Position
        self.Pk = np.zeros((3,3)) # Uncertainty
        
        # Initialize buffer for forcing observing n times a feature before 
        # adding it to the map
        self.featureObservedN = np.array([])
        self.min_observations = 5
    
    #========================================================================
    def get_number_of_features_in_map(self):
        '''
        returns the number of features in the map
        '''
        return (self.xk.size-3)/2
    
    #========================================================================
    def get_polar_line(self, line, odom):
        '''
        Transforms a line from [x1 y1 x2 y2] from the world frame to the
        vehicle frame using odometry [x y ang].
        Returns [range theta]
        '''
        # Line points
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        
        # Compute line (a, b, c) and range
        line = np.array([y1-y2, x2-x1, x1*y2-x2*y1])
        pt = np.array([odom[0], odom[1], 1])
        dist = np.dot(pt, line) / np.linalg.norm(line[:2])
        
        # Compute angle
        if dist < 0:
            ang = np.arctan2(line[1], line[0])
        else:
            ang = np.arctan2(-line[1], -line[0])
        
        # Return in the vehicle frame
        return np.array([np.abs(dist), angle_wrap(ang - odom[2])])
        
    #========================================================================
    def predict(self, uk):
        '''
        Predicts the position of the robot according to the previous position
        and the odometry measurements. It also updates the uncertainty of the
        position
        '''
        #TODO: Program this function
        # - Update self.xk and self.Pk using uk and self.Qk

        # Compute jacobians of the composition with respect to robot (A_k) 
        # and odometry (W_k)
        A = np.array([[1, 0, -np.sin(self.xk[2])*uk[0]-np.cos(self.xk[2])*uk[1]],\
                     [0, 1, +np.cos(self.xk[2])*uk[0]-np.sin(self.xk[2])*uk[1]],\
                     [0, 0, 1]])

        W = np.array([[np.cos(self.xk[2]), -np.sin(self.xk[2]), 0],\
                     [np.sin(self.xk[2]), +np.cos(self.xk[2]), 0],\
                     [0, 0, 1]]) 

        # Prepare the F_k and G_k matrix for the new uncertainty computation
        nf = self.get_number_of_features_in_map()
        F = block_diag(A,np.eye(2*nf))

        G = np.vstack([W,np.zeros([2*nf, 3])])  
 
        #ipdb.set_trace()

        # Compute uncertainty
        self.Pk = np.dot(np.dot(F,self.Pk),(F.T)) + np.dot(np.dot(G,self.Qk),(G.T))

        # Update the class variables
        self.xk[0:3] = comp(self.xk[0:3],uk)
    #========================================================================
        
    def data_association(self, lines):
        '''
        Implements ICNN for each feature of the scan.
        Innovk_List -> matrix containing the innovation for each associated 
                       feature
        H_k_List -> matrix of the jacobians.
        S_f_List -> matrix of the different S_f for each feature associated
        Rk_List -> matrix of the noise of each measurement
        '''
    
        #TODO: Program this function
        # fore each sensed line do:
        #   1- Transform the sened line to polar
        #   2- for each feature of the map (in the state vector) compute the 
        #      mahalanobis distance
        #   3- Data association
        # Variables for finding minimum
        minD = 1e9
        minj = -1
        n = self.get_number_of_features_in_map()
        # Init variable
        Innovk_List   = np.zeros((0,1))
        H_k_List      = np.zeros((0,3+2*n))
        S_f_List      = np.zeros((0,0))
        Rk_List       = np.zeros((0,0))
        idx_not_associated = np.zeros((0,1))

        for i in range(0, lines.shape[0]):

                z = self.get_polar_line(lines[i, :],[0.0,0.0,0.0]);


                for j in range(0, self.get_number_of_features_in_map()):


                        [D,v,h,H,S] = self.lineDist(z, j)
                        islonger = False
                        if np.sqrt(D) < minD and not islonger:
                                minj = j
                                minz = z
                                minh = h    
                                minH = H
                                minv = v
                                minS = S
                                minD = np.sqrt(D)
       
                # Minimum distance below threshold
                if minD < self.chi_thres:
                        print "\t", minz, "->", minh
                        # Append results
                        if self.featureObservedN[j] > self.min_observations :
                            Innovk_List = np.append(Innovk_List,minv)

                            H_k_List  = np.vstack((H_k_List ,minH))

                            S_f_List = np.append(S_f_List,minS)

                            Rk_List = block_diag(Rk_List,self.Rk)
                        else:
                            self.featureObservedN[j] = self.featureObservedN[j]+1

                else:        
                        idx_not_associated = np.append(idx_not_associated,i)
                
        return Innovk_List, H_k_List, S_f_List, Rk_List, idx_not_associated
        
    #========================================================================
    def update_position(self, Innovk_List, H_k_List, Rk_List) :
        '''
        Updates the position of the robot according to the given the position
        and the data association parameters.
        Returns state vector and uncertainty.
        '''
        #TODO: Program this function
        if Innovk_List.shape[0]<1:
            return

        n = self.get_number_of_features_in_map()

        S = Rk_List + np.dot(np.dot(H_k_List,self.Pk),(H_k_List.T))
            
        # Kalman Gain
        K = np.dot(np.dot(self.Pk, (H_k_List.T)), np.linalg.inv(S))
        # Update Position
        self.xk = self.xk + np.dot(K,Innovk_List)
        # Update Uncertainty
        m = np.eye(3+2*n) - np.dot(K, H_k_List)
        self.Pk = np.dot(np.dot(m, self.Pk), m.T) + np.dot(np.dot(K, Rk_List), K.T)   
    #========================================================================
    def state_augmentation(self, lines, idx):
        '''
        given the whole set of lines read by the kineckt sensor and the
        indexes that have not been associated augment the state vector to 
        introduce the new features
        '''
        n = self.get_number_of_features_in_map()
        H_1 = np.eye(3+2*n)
        H_2 = np.zeros((3+2*n,2))
        
        # If no features to add to the map exit function
        # TODO Program this function
        if idx.size<1:
            return 

        for i in range(idx.size):
            z = self.get_polar_line(lines[i, :],[0.0,0.0,0.0])
            [z_tf, H_tf, H_line] = self.tfPolarLine(self.xk[0:3],z)

            self.featureObservedN = np.append(self.featureObservedN,1)

            self.xk = np.append(self.xk,z_tf)
            H_tf = np.hstack((H_tf,np.zeros((2,2*n))))
            H_1 = np.vstack((H_1,H_tf))
            H_2 = np.vstack((H_2,H_line))

            
        self.Pk = np.dot(np.dot(H_1,self.Pk),(H_1.T)) + np.dot(np.dot(H_2,self.Rk),(H_2.T))   

    #========================================================================
    def tfPolarLine(self,tf,line):
        '''
        Transforms a polar line in the robot frame to a polar line in the
        world frame
        '''
        # Decompose transformation
        x_x = tf[0]
        x_y = tf[1]
        x_ang = tf[2]  
        
        # Compute the new phi
        phi = angle_wrap(line[1] + x_ang)
        
        # Auxiliar computations
        sqrt2 = x_x**2+x_y**2
        sqrt = np.sqrt(sqrt2)
        atan2 = np.arctan2(x_y,x_x)
        sin = np.sin(atan2 - phi)
        cos = np.cos(atan2 - phi)
        
        # Compute the new rho
        rho = line[0] + sqrt* cos  
        if rho <0:
            rho = -rho
            phi = angle_wrap(phi+pi)         
        
        # Allocate jacobians
        H_tf = np.zeros((2,3))
        H_line = np.eye(2)

       # TODO: Evaluate jacobian respect to transformation
        H_tf[0,0] = np.cos(phi)
        H_tf[0,1] = np.sin(phi)
        H_tf[0,2] = -x_x*np.sin(phi) + x_y*np.cos(phi)
        H_tf[1,2] = 1

        # TODO: Evaluate jacobian respect to line
        H_line[0,1] = -x_x*np.sin(phi) + x_y*np.cos(phi)
                
        return np.array([rho,phi]), H_tf, H_line
                
    #========================================================================
    def lineDist(self,z,idx):
        '''
        Given a line and an index of the state vector it computes the
        distance between both lines
        '''        
        # TODO program this function
                
        # Transform the map line into robot frame and compute jacobians
        x_rw = self.xk[0:3]

        [tf , J1] = compInv(x_rw)

        [h, H_tf, H_line] = self.tfPolarLine(tf,self.xk[3+2*idx:5+2*idx])

        H_position = np.dot(H_tf,J1)
        
        # Allocate overall jacobian
        nf = self.get_number_of_features_in_map()
        H = np.zeros([2, 3+2*nf])
        
        # Concatenate position jacobians and place them into the position
        H[0:2,0:3] = H_position
        # Place the position of the jacobina with respec to the line in its
        # position
        H[0:2,3+2*idx:5+2*idx] = H_line
        # Calculate innovation
        v = z - h
        
        # Calculate innovation uncertainty
        S = np.dot(np.dot(H,self.Pk),(H.T)) + self.Rk
  
        # Calculate mahalanobis distance
        D = np.dot(np.dot(v.T,np.linalg.inv(S)), v) 
        
        return D,v,h,H,S
