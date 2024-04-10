from pymoo.core.problem import Problem
import numpy as np
from logger import Logger
import bezier_curve
import map_generator
import FRT_path_planner_interface
import error_calculator
import path_planner_interface

def myError(p1x, p2x, p2y, p3x, p3y, logger, path_planner : path_planner_interface):
    # p0 = origo
    p0 = [0, 0]

    #gengerate the curve
    curve = bezier_curve.BezierCurve(p0, [p1x, 0], [p2x, p2y], [p3x, p3y])
    #curve.plot_curve(15)

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
    for i in range(len(curve_points)):
        if curve_points[i][3] > 1/(9-track_width/2):
            #print("too tight")
            error += (track_width/2 + 1) + (curve_points[i][3] - 1/(9-track_width/2)) * 10

    #check if the curve's lenght is in the range of 7-20m
        curve_length = curve.get_curve_length()
    if curve_length < 8:
        #print("too short or long")
        error+= (track_width/2 + 1) + (10 - curve_length)
    
    if curve_length > 20:
        #print("too short or long")
        error += (track_width/2 + 1) + (curve_length - 20)

    if(error > 0):
        #print("error: ", error)
        return error

    #initialize interface with the path planner module
    # path_planner = FRT_path_planner_interface.FRT_path_planner_interface()
    
    #calculate the path
    result = path_planner.calculate_path_on_map(cone_map)

    #calculate the error
    error = error_calculator.calculate_error(result, curve, track_width)

    #log the parameters and the error
    logger.log(p1x, p2x, p2y, p3x, p3y, error)
    #print("error: ", error)
    return error


class TrackProblem(Problem):

    def __init__(self, path_planner : path_planner_interface):
        super().__init__(n_var=5, 
                         n_obj=1, 
                         n_constr=0, 
                         xl=np.array([1, 3, -6, 7.5, -7.5]), 
                         xu=np.array([10, 14, 6, 15, 7.5]),
                         elementwise_evaluation=True)
        self.logger = Logger()
        self.path_planner = path_planner

    def _evaluate(self, x, out, *args, **kwargs):
        n_samples = x.shape[0]
        error = np.empty(n_samples)

        for i in range(n_samples):
            p1x, p2x, p2y, p3x, p3y = x[i, 0], x[i, 1], x[i, 2], x[i, 3], x[i, 4]
            #print (p2, p3, p4)
            error[i] = myError(p1x, p2x, p2y, p3x, p3y, self.logger, self.path_planner)

        out["F"] = error