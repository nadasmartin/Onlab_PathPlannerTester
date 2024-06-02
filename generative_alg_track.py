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
import track_problem_second
import cubic_spline_curve

from logger import Logger
import track_problem_first
import FRT_path_planner_interface
import main 

logFile = "logfile.csv"

FRT_path_planner = FRT_path_planner_interface.FRT_path_planner_interface()
problem = track_problem_second.TrackProblemDegreeDist(FRT_path_planner)

algorithm = NSGA2(
    pop_size=10000,
    n_offsprings=2500,
    sampling=FloatRandomSampling(),
    crossover=SBX(prob=0.9, eta=25),
    mutation=PolynomialMutation(eta=30),
    eliminate_duplicates=True
)

res = minimize(
    problem,
    algorithm,
    ("n_gen", 100),
    verbose=True
)
#print everything about the result
print("X: ", res.X,)
print("F: ",res.F)
print("runtime: ", res.exec_time)


if(res.X.ndim == 1):
    print(res.F)
    #curve = bezier_curve.BezierCurve((0, 0), (res.X[0], 0), (res.X[1], res.X[2]), (res.X[3], res.X[4]))
    p1degree = res.X[0]
    p2degree = res.X[1]
    p2dist = res.X[2]
    p3degree = res.X[3]
    p3dist = res.X[4]
    p0 = [0, 0]
    p1dist = 4
    p1 = [p1dist * np.cos(np.radians(p1degree)), p1dist * np.sin(np.radians(p1degree))]
    p2 = [p1[0] + p2dist * np.cos(np.radians(p2degree)), p1[1] + p2dist * np.sin(np.radians(p2degree))]
    p3 = [p2[0] + p3dist * np.cos(np.radians(p2degree + p3degree)), p2[1] + p3dist * np.sin(np.radians(p2degree + p3degree))]
    curve = cubic_spline_curve.CubicSplineCurve(p0, p1, p2, p3)
else:
    print("F: ", [[f'{f:.9f}' for f in row] for row in res.F])
    sorted_indices = np.argsort(res.F[:, 0])  # Change 0 to another index if you want to sort by a different objective
    # Extract top 10 solutions
    top_10_solution = res.X[sorted_indices[:10]]
    print("Top 10 solution: ", top_10_solution)
    #curve = bezier_curve.BezierCurve((0, 0), (top_10_solution[0][0], 0), (top_10_solution[0][1], top_10_solution[0][2]), (top_10_solution[0][3], top_10_solution[0][4]))
    p1degree = top_10_solution[0][0]
    p2degree = top_10_solution[0][1]
    p2dist = top_10_solution[0][2]
    p3degree = top_10_solution[0][3]
    p3dist = top_10_solution[0][4]
    p0 = [0, 0]
    p1dist = 4
    p1 = [p1dist * np.cos(np.radians(p1degree)), p1dist * np.sin(np.radians(p1degree))]
    p2 = [p1[0] + p2dist * np.cos(np.radians(p2degree)), p1[1] + p2dist * np.sin(np.radians(p2degree))]
    p3 = [p2[0] + p3dist * np.cos(np.radians(p2degree + p3degree)), p2[1] + p3dist * np.sin(np.radians(p2degree + p3degree))]
    curve = cubic_spline_curve.CubicSplineCurve(p0, p1, p2, p3)

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

