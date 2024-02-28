import bezier_curve
import map_generator
import FRT_path_planner_interface
import time
import matplotlib.pyplot as plt
import error_calculator

def plotAllCurves(curve, cone_map, result):
    # Plot the curve
    curve_points = curve.get_curve(100)  # Get 100 points along the curve
    # Separate the x and y coordinates
    curve_xs, curve_ys = zip(*[(point[0], point[1]) for point in curve_points])
    plt.plot(curve_xs, curve_ys, label='Curve')

    # Plot the cone_map
    cone_xs, cone_ys, cone_colors = zip(*cone_map)  # Separate the x, y coordinates and colors
    plt.scatter(cone_xs, cone_ys, c=cone_colors, label='Cones')

    # Plot the result
    # Assuming result is a list of (x, y) tuples
    if result:
        result_xs, result_ys = zip(*result)  # Separate the x and y coordinates
        plt.plot(result_xs, result_ys, label='Result')

    plt.axis('equal')  # Set the aspect ratio of the plot to be equal
    plt.legend()  # Add a legend
    plt.show()  # Display the plot


curve = bezier_curve.BezierCurve((1, 0), (15, 0), (20, 20), (0, 20))
curve.plot_curve(15)

map = map_generator.Map(curve)
map.plot_map()
cone_map = map.get_cones()

path_planner = FRT_path_planner_interface.FRT_path_planner_interface()
result = path_planner.calculate_path_on_map(cone_map)

# plot the result, cone_map and the curve
plotAllCurves(curve, cone_map, result)

# Calculate the error
error = error_calculator.calculate_error(curve, result)








