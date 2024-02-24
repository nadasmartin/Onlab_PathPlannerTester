import numpy as np
import matplotlib.pyplot as plt
from threading import Timer

class BezierCurve:
    def __init__(self, P0, P1, P2, P3):
        self.P0 = P0
        self.P1 = P1
        self.P2 = P2
        self.P3 = P3

    def _bezier_point(self, t, axis):
        return (
            (1-t)**3 * self.P0[axis] +
            3 * (1-t)**2 * t * self.P1[axis] +
            3 * (1-t) * t**2 * self.P2[axis] +
            t**3 * self.P3[axis]
        )

    def _bezier_derivative_y(self, t):
        return (
            -3 * self.P0[1] * (1-t)**2 +
            3 * self.P1[1] * (1-t)**2 - 6 * t * self.P1[1] * (1-t) +
            6 * t * self.P2[1] * (1-t) - 3 * t**2 * self.P2[1] +
            3 * t**2 * self.P3[1]
        )

    def _newton_raphson(self, target_y, initial_guess=0.5, iterations=10):
        t = initial_guess
        for _ in range(iterations):
            y = self._bezier_point(t, 1)
            y_prime = self._bezier_derivative_y(t)
            
            if y_prime == 0:  # Avoid division by zero
                break
                
            t = t - (y - target_y) / y_prime

        return t

    def get_x_for_y(self, y):
        t = self._newton_raphson(y)
        return self._bezier_point(t, 0)

    def plot_curve(self, n_points=25):
        curve_points = self.get_curve(n_points)
        x_values = [point[0] for point in curve_points]
        y_values = [point[1] for point in curve_points]

        # x_for_target_y = self.get_x_for_y(target_y)

        plt.figure(figsize=(8, 6))
        #plt.plot(x_values, y_values, label="Bezier Curve", color='blue')
        plt.scatter(x_values, y_values, color='blue', label="Bezier Curve")
        # plt.scatter([x_for_target_y], [target_y], color='red', zorder=5)
        # plt.text(x_for_target_y, target_y, f"({x_for_target_y:.2f}, {target_y})", fontsize=10, verticalalignment='bottom')
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)

        #plot control points as well
        plt.scatter([self.P0[0], self.P1[0], self.P2[0], self.P3[0]], [self.P0[1], self.P1[1], self.P2[1], self.P3[1]], color='red', label="Control Points")
        #plot orientation and curvature
        curve_points = self.get_curve(n_points)
        x_values = [point[0] for point in curve_points]
        y_values = [point[1] for point in curve_points]
        orientation = [point[2] for point in curve_points]
        curvature = [point[3] for point in curve_points]
        plt.quiver(x_values, y_values, np.cos(orientation), np.sin(orientation), color='green', label="Orientation")
        #write curvature on the plot
        for i in range(len(x_values)):
            plt.text(x_values[i], y_values[i], f"{curvature[i]:.2f}", fontsize=8, verticalalignment='bottom', horizontalalignment='right')


        plt.legend()
        plt.show()

    #returns a list of points on the curve, n_patiotions is the number of points equally spaced on the curve, each point is a tuple (x, y, orientation, curvature)
    def get_curve(self, n_patiotions):
        curve_points = []
        t_values = np.linspace(0, 1, n_patiotions)
        for t in t_values:
            x = self._bezier_point(t, 0)
            y = self._bezier_point(t, 1)

            # Calculate the first and second derivatives of the curve
            x_prime = 3*(1-t)**2*(P1[0]-P0[0]) + 6*(1-t)*t*(P2[0]-P1[0]) + 3*t**2*(P3[0]-P2[0])
            y_prime = 3*(1-t)**2*(P1[1]-P0[1]) + 6*(1-t)*t*(P2[1]-P1[1]) + 3*t**2*(P3[1]-P2[1])

            x_double_prime = 6*(1-t)*(P2[0]-2*P1[0]+P0[0]) + 6*t*(P3[0]-2*P2[0]+P1[0])
            y_double_prime = 6*(1-t)*(P2[1]-2*P1[1]+P0[1]) + 6*t*(P3[1]-2*P2[1]+P1[1])

            # Calculate the curvature
            curvature = np.abs(x_prime*y_double_prime - y_prime*x_double_prime) / (x_prime**2 + y_prime**2)**1.5

            # Calculate the orientation
            orientation = np.arctan2(y_prime, x_prime)

            # Append the point to the list
            curve_points.append((x, y, orientation, curvature))

        return curve_points

        

        
        
        
        

# Bezier curve parameters
P0 = (0, 0)
P1 = (0, 70)
P2 = (50, 55)
P3 = (90, 0)

# Create the Bezier curve
bezier_curve = BezierCurve(P0, P1, P2, P3)

# Plot the Bezier curve
bezier_curve.plot_curve(15)

