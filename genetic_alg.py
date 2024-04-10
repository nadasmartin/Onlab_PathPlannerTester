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

import FRT_path_planner_interface
from problems import TrackProblem

logFile = "logfile.csv"

path_planner_interface = FRT_path_planner_interface.FRT_path_planner_interface()
problem = TrackProblem(path_planner_interface)

algorithm = NSGA2(
    pop_size=10,
    n_offsprings=10,
    sampling=FloatRandomSampling(),
    crossover=SBX(prob=0.9, eta=15),
    mutation=PolynomialMutation(eta=20),
    eliminate_duplicates=True
)

res = minimize(
    problem,
    algorithm,
    ("n_gen", 150),
    verbose=True
)

#sort the results by the first objective 

sorted_indices = np.argsort(res.F[:, 0])  # Change 0 to another index if you want to sort by a different objective

# Extract top 10 solutions
top_10_solutions = res.X[sorted_indices[:10]]
