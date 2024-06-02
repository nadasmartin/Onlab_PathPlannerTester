import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

class CubicSplineCurve():
    def __init__(self, P0, P1, P2, P3):
        self.P0 = P0
        self.P1 = P1
        self.P2 = P2
        self.P3 = P3
        self.t = np.array([0, 1, 2, 3])
        self.x = np.array([P0[0], P1[0], P2[0], P3[0]])
        self.y = np.array([P0[1], P1[1], P2[1], P3[1]])
        self.cs_x = CubicSpline(self.t, self.x)
        self.cs_y = CubicSpline(self.t, self.y)

    #returns the length of the curve
    def get_curve_length(self, n_partitions = 1000):
        t = np.linspace(0, 3, n_partitions)
        dx_dt = self.cs_x(t, 1)
        dy_dt = self.cs_y(t, 1)
        return np.sum(np.sqrt(dx_dt**2 + dy_dt**2)) * (3 / n_partitions)

    #returns the curve points as list of tuples (x, y, orientation, curvature, distance)
    def get_curve(self, n_partitions):
        curve_points = []
        t = np.linspace(0, 3, n_partitions)
        x = self.cs_x(t)
        y = self.cs_y(t)
        dx_dt = self.cs_x(t, 1)
        dy_dt = self.cs_y(t, 1)
        d2x_dt2 = self.cs_x(t, 2)
        d2y_dt2 = self.cs_y(t, 2)
        orientation = np.arctan2(dy_dt, dx_dt)
        curvature = np.abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / (dx_dt**2 + dy_dt**2)**1.5
        distance = np.cumsum(np.sqrt(dx_dt**2 + dy_dt**2)) * (3 / n_partitions)
        for i in range(x.shape[0]):
            curve_points.append((x[i], y[i], orientation[i], curvature[i], distance[i]))
        return curve_points
    
    def plot_curve(self, n_partitions=1000):
        curve_points = self.get_curve(n_partitions)
        x, y, _, _, _ = zip(*curve_points)

        plt.figure(figsize=(8, 6))
        plt.plot(x, y, label='Cubic Spline Curve')
        plt.scatter([self.P0[0], self.P1[0], self.P2[0], self.P3[0]], 
                    [self.P0[1], self.P1[1], self.P2[1], self.P3[1]], 
                    color='red', label='Control Points')
        plt.title('Cubic Spline Curve')
        
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

curve = CubicSplineCurve((0, 0), (3.9316527821897433, 0.736278751764338), (7.396301993786235, 2.665424142910973), (10.11575203086888, 5.5476176005836315))
curve.plot_curve()
print(curve.get_curve_length())