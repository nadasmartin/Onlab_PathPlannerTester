import curve
import matplotlib.pyplot as plt
import numpy as np
from typing import Type

#this class is used to generate a cone map from a curve
class Map:
    def __init__(self, curve: Type[curve.Curve], track_width = 3.0):
        
        self.curve = curve
        self.curve_length = curve.get_curve_length()
        self.track_width = track_width
        #every 3 units of curve length, a cone is placed on both sides of the curve with a track width of track_width
        self.curve_points = curve.get_curve(int(self.curve_length/3))
        self.cones = self.generate_cones()

    #generates the cones for the map
    def generate_cones(self):
        cones = []
        for point in self.curve_points:
            x = point[0]
            y = point[1]
            orientation = point[2]
            curvature = point[3]
            distance = point[4]

            # a cone is placed on the left and right side of the curve at a distance of track_width/2
            # every cone is placed perpendicular to the orientation of the curve
             #cones on the left are blue, cones on the right are yellow
            # left cone
            left_cone_x = x + (self.track_width/2)*np.cos(orientation + np.pi/2)
            left_cone_y = y + (self.track_width/2)*np.sin(orientation + np.pi/2)
            cones.append((left_cone_x, left_cone_y, "blue"))
            # right cone
            right_cone_x = x - (self.track_width/2)*np.cos(orientation + np.pi/2)
            right_cone_y = y - (self.track_width/2)*np.sin(orientation + np.pi/2)
            cones.append((right_cone_x, right_cone_y, "yellow"))

        return cones
    
    #plots the cones on the map
    def plot_map(self):
        for cone in self.cones:
            plt.plot(cone[0], cone[1], 'o', color = cone[2])
            plt.grid(True)
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.title("Map")
            plt.legend(["Left Cone", "Right Cone"])
        plt.show()

    def get_cones(self):
        return self.cones