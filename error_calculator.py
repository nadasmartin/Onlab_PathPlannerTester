import numpy as np
import matplotlib.pyplot as plt
import curve

#this function calculates the error between the caculated and actual ground truth curve
def calculate_error(path, ground_truth_curve : curve.Curve):

    #get the curve points from ground truth curve every approximatelly 1 meter
    curve_points = ground_truth_curve.get_curve(int(ground_truth_curve.get_curve_length()))
    for cp in range(len(curve_points)):
        error += np.linalg.norm(np.array(curve_points[i]) - np.array(cp))
    return error