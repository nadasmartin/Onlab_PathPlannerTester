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

import FRT_path_planner_interface
import main
from problems_cone import ConeProblem

FRT_path_planner = FRT_path_planner_interface.FRT_path_planner_interface()

curve = bezier_curve.BezierCurve((0, 0), (3.86, 0), (11.65, 2.23), (14.42, -7.07))
problem = ConeProblem(FRT_path_planner, curve)

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
print(res.X)

# There is only one solution:
if(res.X.ndim == 1):
    print(res.F)
    map = map_generator.Map(curve, res.X[0], res.X[1], res.X[2], res.X[3])
else:
    print("F: ", [[f'{f:.9f}' for f in row] for row in res.F])
    sorted_indices = np.argsort(res.F[:, 0])  # Change 0 to another index if you want to sort by a different objective
    # Extract top 10 solutions
    top_10_solution = res.X[sorted_indices[:10]]
    print("Top 1 solution: ", top_10_solution)
    map = map_generator.Map(curve, top_10_solution[0][0], top_10_solution[0][1], top_10_solution[0][2], top_10_solution[0][3])

cone_map = map.generate_cones()
#map.plot_map()

path_planner = FRT_path_planner_interface.FRT_path_planner_interface()
result = path_planner.calculate_path_on_map(cone_map)

# plot the result, cone_map and the curve
main.plotAllCurves(curve, cone_map, result, "cone_placement")

# sorted_indices = np.argsort(res.F[:, 0])  # Change 0 to another index if you want to sort by a different objective

# # Extract top 10 solutions
# top_10_solutions = res.X[sorted_indices[:10]]

