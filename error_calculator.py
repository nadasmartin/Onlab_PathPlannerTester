import numpy as np
import matplotlib.pyplot as plt
import curve
from scipy.spatial import KDTree

#this function calculates the error between the caculated and actual ground truth curve
def calculate_error(path, ground_truth_curve : curve.Curve, track_width : float):

    # Number of points to sample from the ground truth curve, approximatelly every 1 meter
    n_samples = int(ground_truth_curve.get_curve_length())

    #get the curve points from ground truth curve 
    curve_points = ground_truth_curve.get_curve(n_samples)

            
    # Create a k-d tree from the path points
    path_tree = KDTree(path)
    
    have_left_track = False
    total_error = 0
    for i in range(len(curve_points)):
        # Extract the x and y coordinates
        x, y, orientation, curvature, d = curve_points[i]
        # Query the KDTree using only the x and y coordinates
        distance, index = path_tree.query([x, y])
        total_error += distance
        if(distance >= track_width/2):
            have_left_track = True
    
    # the lower the return_error, the worst the path is
    if(have_left_track):
        # because path left the track, we give it minimum
        return 0
    else:
        # the path is on the track, we average the error and subtract it from the track width
        return track_width/2 - total_error/n_samples