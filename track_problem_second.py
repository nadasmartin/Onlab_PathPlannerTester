from pymoo.core.problem import Problem
import numpy as np
from logger import Logger
import bezier_curve
import map_generator
import FRT_path_planner_interface
import error_calculator
import path_planner_interface
import cubic_spline_curve

def myError(p1degree, p2degree, p2dist, p3degree, p3dist, logger, path_planner : path_planner_interface):
    # p0 = origo
    p0 = [0, 0]
    p1dist = 4
    p1 = [p1dist * np.cos(np.radians(p1degree)), p1dist * np.sin(np.radians(p1degree))]
    p2 = [p1[0] + p2dist * np.cos(np.radians(p1degree + p2degree)), p1[1] + p2dist * np.sin(np.radians(p1degree + p2degree))]
    p3 = [p2[0] + p3dist * np.cos(np.radians(p1degree + p2degree + p3degree)), p2[1] + p3dist * np.sin(np.radians(p1degree + p2degree + p3degree))]

    #gengerate the curve
    curve = cubic_spline_curve.CubicSplineCurve(p0, p1, p2, p3)
    #curve.plot_curve(50)

    #generate the map
    track_width = 3.0
    map = map_generator.Map(curve, track_width)
    cone_map = map.generate_cones()
    #map.plot_map()

    # Number of points to sample from the ground truth curve, approximatelly every 1 meter
    n_samples = int(curve.get_curve_length()) 

    #get the curve points from ground truth curve 
    curve_points = curve.get_curve(n_samples)
    error = 0
    #check if curvature is always smaller than 1/radius, where radius is minimum 9m on the outer side of the track
    greatest_curvature = 0
    for i in range(len(curve_points)):
        if curve_points[i][3] > 1/(9-track_width/2):
            #print("too tight")
            if(greatest_curvature < curve_points[i][3]):
                greatest_curvature = curve_points[i][3]
            error += (track_width/2 + 1)

    curvature_constraint = greatest_curvature - (1/(9-track_width/2))

    #check if the curve's lenght is in the range of 7-20m
    curve_length = curve.get_curve_length()
    if curve_length < 8:
        #print("too short or long")
        error+= (track_width/2 + 1)
    
    if curve_length > 20:
        #print("too short or long")
        error += (track_width/2 + 1)

    curve_length_constraint = np.abs(curve_length - 15) - 7
    
    if(error > 0):
        # print("no path error: ", error)
        return [10* error, curvature_constraint, curve_length_constraint]
    
    #initialize interface with the path planner module
    # path_planner = FRT_path_planner_interface.FRT_path_planner_interface()
    
    #calculate the path
    result = path_planner.calculate_path_on_map(cone_map)

    #calculate the error
    error = error_calculator.calculate_error(result, curve, track_width)
    # print("error: ", error)
    #log the parameters and the error
    logger.log(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], error)
    #print("error: ", error)
    return [error, curvature_constraint, curve_length_constraint]


class TrackProblemDegreeDist(Problem):

    def __init__(self, path_planner : path_planner_interface):
        super().__init__(n_var=5, 
                         n_obj=1, 
                         n_ieq_constr=2,
                         xl=np.array([-20, -75, 3, -75, 3]), 
                         xu=np.array([20, 75, 8, 75, 8]),
                         elementwise_evaluation=True)
        self.logger = Logger()
        self.path_planner = path_planner

    def _evaluate(self, x, out, *args, **kwargs):
        n_samples = x.shape[0]
        error = np.empty(n_samples)
        curvature_constraint = np.empty(n_samples)
        curve_length_constraint = np.empty(n_samples)

        for i in range(n_samples):
            p1x, p2degree, p2dist, p3degree, p3dist = x[i, 0], x[i, 1], x[i, 2], x[i, 3], x[i, 4]
            #print (p2, p3, p4)
            [e, curvature_const, curve_length_const] = myError(p1x, p2degree, p2dist, p3degree, p3dist, self.logger, self.path_planner)
            error[i] = e
            curvature_constraint[i] = curvature_const
            curve_length_constraint[i] = curve_length_const
        # print (error)
        out["F"] = error
        out["G"] = np.column_stack([curvature_constraint, curve_length_constraint])