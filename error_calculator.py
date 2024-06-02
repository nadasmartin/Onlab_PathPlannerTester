import numpy as np
import matplotlib.pyplot as plt
import curve
from scipy.spatial import KDTree

#this function calculates the error between the caculated and actual ground truth curve
def calculate_error(path, ground_truth_curve : curve.Curve, track_width = 3.0):
    # Convert path to numpy array and create KDTree
    path = np.array(path)
    tree = KDTree(path)

    # Get the lenght of the curve
    n_samples = int(ground_truth_curve.get_curve_length())
    
    #get the curve points from ground truth curve every approximatelly 1 meter
    curve_points = ground_truth_curve.get_curve(n_samples)
    total_error = 0
    have_left_track = False
    for cp in curve_points:
        # cp is a point on the curve and has x,y, orientation, curvature and distance information
        curvepoint = np.array([cp[0], cp[1]])
        # Query the KDTree to find the closest path point
        dist, idx = tree.query(curvepoint)
        # Calculate the error
        error = np.linalg.norm(curvepoint - path[idx])
        if(error > track_width/2):
            have_left_track = True
        total_error += error
        
    if(have_left_track):
        print("The car has left the track!!!")
        #minimum error is 0
        return 0
    else:
        return 10*(track_width/2 - (total_error / n_samples))