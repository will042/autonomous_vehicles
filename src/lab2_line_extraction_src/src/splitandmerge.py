#!/usr/bin/python
# -*- coding: utf-8 -*-

#Christian Fontent & William Ard
import numpy as np
import math
from probabilistic_lib.functions import angle_wrap

#===============================================================================
def splitandmerge(points, split_thres=0.1, inter_thres=0.3, min_points=6, dist_thres=0.12, ang_thres=np.deg2rad(10)):
    '''
    Takes an array of N points in shape (2, N) being the first row the x
    position and the second row the y position.

    Returns an array of lines of shape (L, 4) being L the number of lines, and
    the columns the initial and final point of each line in the form
    [x1 y1 x2 y2].

    split_thres: distance threshold to provoke a split
    inter_thres: maximum distance between consecutive points in a line
    min_point  : minimum number of points in a line
    dist_thres : maximum distance to merge lines
    ang_thres  : maximum angle to merge lines
    '''
    lines = split(points, split_thres, inter_thres, min_points, 0, points.shape[1]-1)
    return merge(lines, dist_thres, ang_thres)
    
#===============================================================================
def split(points, split_thres, inter_thres, min_points, first_pt, last_pt):
    '''
    Find lines in the points provided.
    first_pt: column position of the first point in the array
    last_pt : column position of the last point in the array
    '''
    assert first_pt >= 0
    assert last_pt <= points.shape[1]
    
    # TODO: CODE HERE!!!
    # Check minimum number of points
    if last_pt-first_pt + 1 < min_points:
        return None

    # Line defined as "a*x + b*y + c = 0"
    # Calc (a, b, c) of the line (prelab question)
    x1 = points[0, first_pt]
    y1 = points[1, first_pt]
    x2 = points[0, last_pt]
    y2 = points[1, last_pt]
    values = [y1-y2, x2-x1, x1*y2-x2*y1] #Values holds a,b,c for standard form of a line


    #distance of points to line
    dist = np.zeros(last_pt+1-first_pt)

    #calculate distance of each point to line, then get index of biggest
    for i in range(first_pt, last_pt+1):
        dist[i-first_pt] = (abs(values[0]*points[0,i] + values[1]*points[1,i] + values[2]) / math.sqrt(values[0]*values[0] + values[1]*values[1]))
    maxdist_index = np.argmax(dist)

    # Check split threshold
    if dist[maxdist_index] > split_thres:
       
        # Check sublines
        # maxdist_index is not directly an index of points, it is offset by first_pt
        prev = split(points, split_thres, inter_thres, min_points, first_pt, first_pt+maxdist_index)
        post = split(points, split_thres, inter_thres, min_points, first_pt+maxdist_index, last_pt)
       
        # Return results of sublines
        if prev is not None and post is not None:
            #both partitions yield valid lines, so return both
            return np.vstack((prev, post))
        elif prev is not None:
            return prev
        elif post is not None:
            return post
        else:
            return None

    # # Do not need to split furthermore
    else:
        #This code is simplified as the partitions are done so that they share an endpoint, so
        # no intermediate line is necessary
        return np.array([[x1,y1,x2,y2]])
        # Optional check interpoint distance
        #for i in range(first_pt, last_pt):
            #
            #
            #
            #
            # Check interpoint distance threshold
            #if ... > inter_thres:
                #Split line
                #prev = split(points, split_thres, inter_thres, min_points, ...)
                #post = split(points, split_thres, inter_thres, min_points, ...)
               
                # Return results of sublines
                #if prev is not None and post is not None:
                    #return np.vstack((prev, post))
                #elif prev is not None:
                    #return prev
                #elif post is not None:
                    #return post
                #else:
                    #return None
        
        # It is a good line
        #return np.array([[x1, y1, x2, y2]])
        
    # Dummy answer --> delete it
    #return np.array([[x1, y1, x2, y2]])

#===============================================================================
def merge(lines, dist_thres, ang_thres):
    '''
    Merge similar lines according to the given thresholds.
    '''

    # No data received
    if lines is None:
        return np.array([])
        
    # # Check and merge similar consecutive lines
    i = 0

    #While is based on i+1 as the calculation relies on 2 lines, not just i, but the following
    while i+1 < np.size(lines, axis=0):

        # Line angles - get angle for each endpoint
        ang1 = math.atan2(lines[i,0],lines[i,1]) 
        ang2 = math.atan2( lines[i+1,2], lines[i+1,3])
        
        # Below thresholds?
        angdiff = abs(ang2 - ang1)

        # name points to simplify formula for disdiff
        x1a = lines[i,2]
        y1a = lines[i,3]
        x2b = lines[i+1,0]
        y2b = lines[i+1,1]

        disdiff = math.sqrt((x2b-x1a)**2 + (y2b-y1a)**2)
        if angdiff < ang_thres and disdiff < dist_thres:
            
            # Joined line
            lines[i,:] = [x1a, y1a, x2b, y2b]
            print("Merging Lines")
            # Delete unnecessary line
            lines = np.delete(lines, i+1, axis=0)
            
        # Nothing to merge
        else:
            i += 1 #only increment if nothing merged, as it may be possible to merge several successive lines
            
    return lines
