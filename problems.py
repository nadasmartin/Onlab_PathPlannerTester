from pymoo.core.problem import Problem
import numpy as np
import math
from logger import Logger
import path_planner_interface
import bezier_curve
import map_generator
import error_calculator

def myError(b, c, d, logger, path_planner):

    p1 = (0,0)
    p2 = (0,5)
    p3 = (c, b)
    p4 = (d, 200)

    # Generate a curve
    curve = bezier_curve.BezierCurve(p1, p2, p3, p4)

    # Generate a map
    map = map_generator.Map(curve)
    cone_map = map.get_cones()

    # Calculate the path
    result = path_planner.calculate_path_on_map(cone_map)

    # Calculate the error
    error = error_calculator.calculate_error(result, curve)

    # Log the parameters and the error
    logger.log(b, c, d, error)

    return error


class TrackProblem(Problem):

    def __init__(self, path_planner: path_planner_interface.path_planner_interface):
        super().__init__(n_var=3, 
                         n_obj=1, 
                         n_constr=0, 
                         xl=np.array([70, -40, -60]), 
                         xu=np.array([150, 40, 60]),
                         elementwise_evaluation=True)
        self.path_planner = path_planner
        self.logger = Logger()

    def _evaluate(self, x, out, *args, **kwargs):
        n_samples = x.shape[0]
        error = np.empty(n_samples)

        for i in range(n_samples):
            b, c, d = x[i, 0], x[i, 1], x[i, 2]
            error[i] = myError(b, c, d, self.logger, self.path_planner)

        out["F"] = error.reshape(-1, 1)