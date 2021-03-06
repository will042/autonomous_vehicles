#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
from probabilistic_lib.functions import angle_wrap, get_polar_line
from math import exp

#===============================================================================
class ParticleFilter(object):
    '''
    Class to hold the whole particle filter.
    
    p_wei: weights of particles in array of shape (N,)
    p_ang: angle in radians of each particle with respect of world axis, shape (N,)
    p_xy : position in the world frame of the particles, shape (2,N)
    '''
    
    #===========================================================================
    def __init__(self, room_map, num, odom_lin_sigma, odom_ang_sigma, 
                 meas_rng_noise, meas_ang_noise,x_init,y_init,theta_init):
        '''
        Initializes the particle filter
        room_map : an array of lines in the form [x1 y1 x2 y2]
        num      : number of particles to use
        odom_lin_sigma: odometry linear noise
        odom_ang_sigma: odometry angular noise
        meas_rng_noise: measurement linear noise
        meas_ang_noise: measurement angular noise
        '''
        
        # Copy parameters
        self.map = room_map
        self.num = num
        self.odom_lin_sigma = odom_lin_sigma
        self.odom_ang_sigma = odom_ang_sigma
        self.meas_rng_noise = meas_rng_noise
        self.meas_ang_noise = meas_ang_noise
        
        # Map
        map_xmin = np.min(self.map[:, 0])
        map_xmax = np.max(self.map[:, 0])
        map_ymin = np.min(self.map[:, 1])
        map_ymax = np.max(self.map[:, 1])
        
        # Particle initialization arround starting point
        self.p_wei = 1.0 / num * np.ones(num)
        self.p_ang =2 * np.pi * np.random.rand(num)
        self.p_xy  = np.vstack(( x_init+ 1*np.random.rand(num)-0.5,
                                 y_init+ 1*np.random.rand(num)-0.5 ))
        #Flags for resampling                         
        self.moving=False
        self.n_eff=0 #Initialize Efficent number as 0
    
    #===========================================================================
    def predict(self, odom):
        '''
        Moves particles with the given odometry.
        odom: incremental odometry [delta_x delta_y delta_yaw] in the vehicle frame
        '''
        #Check if we have moved from previous reading.
        if odom[0]==0 and odom[1]==0 and odom[2]==0:
            self.moving=False
        else:
            # TODO: code here!!

            # Add Gaussian noise to odometry measures
            self.odom_lin_noise = np.random.randn(2,self.num) * self.odom_lin_sigma
            self.odom_ang_noise = np.random.randn(self.num,) * self.odom_ang_sigma
            

            # Increment particle positions in correct frame
            self.p_xy[0] += np.cos(self.p_ang) * odom[0] - np.sin(self.p_ang) * odom[1] + self.odom_lin_noise[0]
            self.p_xy[1] += np.cos(self.p_ang) * odom[1] + np.sin(self.p_ang) * odom[0] + self.odom_lin_noise[1]


            # Increment angle
            self.p_ang += odom[2]+self.odom_ang_noise
            self.p_ang = angle_wrap(self.p_ang)

            #Update flag for resampling
            self.moving=True     


    #===========================================================================
    def weight(self, lines):
        '''
        Look for the lines seen from the robot and compare them to the given map.
        Lines expressed as [x1 y1 x2 y2].
        '''
        # TODO: code here!!
        # Constant values for all weightings
        val_rng = 1.0 / (self.meas_rng_noise * np.sqrt(2 * np.pi))
        val_ang = 1.0 / (self.meas_ang_noise * np.sqrt(2 * np.pi))

        # Loop over particles
        for i in range(self.num):
            
            # Transform map lines to local frame and to [range theta]
            lines_map = np.zeros([self.map.shape[0], 2])

            for j in range(self.map.shape[0]):
                lines_map[j,:] = get_polar_line(self.map[j], [self.p_xy[0,i], self.p_xy[1,i], self.p_ang[i]])



            # Transform measured lines to [range theta] and weight them
            for j in range(lines.shape[0]):
                measured_lines = get_polar_line(lines[j, :])

            # Weight them
            weight = np.zeros([lines_map.shape[0],1])
            for j in range(lines_map.shape[0]):
                weight_rng = exp(-1*(measured_lines[0] - lines_map[j,0])**2/(2*self.meas_rng_noise**2))*val_rng
                weight_ang = exp(-1*(measured_lines[1] - lines_map[j,1])**2/(2*self.meas_ang_noise**2))*val_ang
                weight[j] = weight_rng * weight_ang

            self.p_wei[i] = self.p_wei[i] * np.max(weight)

            # OPTIONAL question
            # make sure segments correspond, if not put weight to zero
            #
            #
            
            # Take best weighting (best associated lines)
            #
            
        # Normalize weights
        self.p_wei /= np.sum(self.p_wei)
        # TODO: Compute efficient number
        self.n_eff=0.0
        
    #===========================================================================
    def resample(self):
        '''
        Systematic resampling of the particles.
        '''
        # TODO: code here!!
        # Look for particles to replicate

        xy = np.zeros([2, self.num])
        ang = np.zeros([self.num,])

        
        r = np.random.rand() / self.num
        c = self.p_wei[0]
        i = 0

        for m in range(self.num):
        
            u = r + float(m) /self.num
        
            while u > c:
                i += 1
                c += self.p_wei[i]
        
            xy[:,m] = self.p_xy[:,i]
            ang[m] = self.p_ang[i]
            
        # Pick chosen particles

        self.p_xy = xy
        self.p_ang = ang
        self.p_wei = np.ones(self.num) / self.num

    
    #===========================================================================
    def get_mean_particle(self):
        '''
        Gets mean particle.
        '''
        # Weighted mean
        weig = np.vstack((self.p_wei, self.p_wei))
        mean = np.sum(self.p_xy * weig, axis=1) / np.sum(self.p_wei)
        
        ang = np.arctan2( np.sum(self.p_wei * np.sin(self.p_ang)) / np.sum(self.p_wei),
                          np.sum(self.p_wei * np.cos(self.p_ang)) / np.sum(self.p_wei) )
                          
        return np.array([mean[0], mean[1], ang])
