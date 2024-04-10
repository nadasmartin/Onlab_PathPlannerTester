from pymoo.core.problem import Problem
import numpy as np
import bezier_curve
from path_planner_interface import path_planner_interface
import map_generator
import error_calculator





class ConeProblem(Problem):
    def __init__(self, path_planner : path_planner_interface, curve : bezier_curve.BezierCurve):
        super().__init__(n_var=4, 
                         n_obj=1, 
                         n_constr=0, 
                         xl=np.array([0.7, 0.7, 0.05, 3]), 
                         xu=np.array([5, 5, 0.25, 5]),
                         elementwise_evaluation=True)
        self.path_planner = path_planner
        self.curve = curve

    def myError(self, right_cone_separation, left_cone_separation, std_deviation, track_width):
        
        map = map_generator.Map(self.curve, right_cone_separation, left_cone_separation, std_deviation, track_width)
        cone_map = map.generate_cones()
        #map.plot_map()

        # Number of points to sample from the ground truth curve, approximatelly every 1 meter
        n_samples = int(self.curve.get_curve_length())

        error = 0
        
        #calculate the path
        result = self.path_planner.calculate_path_on_map(cone_map)

        #calculate the error
        error = error_calculator.calculate_error(result, self.curve, track_width)

        #print("error: ", error)
        return error

    def _evaluate(self, x, out, *args, **kwargs):
        n_samples = x.shape[0]
        error = np.empty(n_samples)

        for i in range(n_samples):
            right_cone_separation, left_cone_separation, std_deviation, track_width = x[i, 0], x[i, 1], x[i, 2], x[i, 3]
            error[i] = self.myError(right_cone_separation, left_cone_separation, std_deviation, track_width)

        out["F"] = error