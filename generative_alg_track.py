import math
import os

import matplotlib.pyplot as plt
import numpy as np
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.factory import get_crossover, get_mutation
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PolynomialMutation
from pymoo.operators.sampling.rnd import FloatRandomSampling
from pymoo.optimize import minimize
import bezier_curve
import map_generator


from logger import Logger
from problems_track import TrackProblem
import FRT_path_planner_interface
import main

logFile = "logfile.csv"

FRT_path_planner = FRT_path_planner_interface.FRT_path_planner_interface()
problem = TrackProblem(FRT_path_planner)

algorithm = NSGA2(
    pop_size=35,
    n_offsprings=15,
    sampling=FloatRandomSampling(),
    crossover=SBX(prob=0.9, eta=15),
    mutation=PolynomialMutation(eta=20),
    eliminate_duplicates=True
)

res = minimize(
    problem,
    algorithm,
    ("n_gen", 50),
    verbose=True
)
#print everything about the result
print("X: ", res.X,)
print("F: ",res.F)

curve = bezier_curve.BezierCurve((0, 0), (res.X[0], 0), (res.X[1], res.X[2]), (res.X[3], res.X[4]))

track_width = 3.0

map = map_generator.Map(curve, track_width)
cone_map = map.generate_cones()
path_planner = FRT_path_planner_interface.FRT_path_planner_interface()
result = path_planner.calculate_path_on_map(cone_map)

# plot the result, cone_map and the curve
main.plotAllCurves(curve, cone_map, result, "track_placement")

# sorted_indices = np.argsort(res.F[:, 0])  # Change 0 to another index if you want to sort by a different objective

# # Extract top 10 solutions
# top_10_solutions = res.X[sorted_indices[:10]]

