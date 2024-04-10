import curve
import matplotlib.pyplot as plt
import numpy as np
from typing import Type


#this class is used to generate a cone map from a curve
class Map:
    def __init__(self, curve: Type[curve.Curve], right_cone_separation = 3.0, left_cone_separation = 3.0, std_deviation = 0.001, track_width = 3.0,):
        
        self.curve = curve
        self.right_cone_separation = right_cone_separation
        self.left_cone_separation = left_cone_separation
        self.std_deviation = std_deviation
        self.track_width = track_width
        self.cones = []

    #generates the cones for the map
    def generate_cones(self):
        self.cones.clear()
        curve_points = self.curve.get_curve(1000)

        # calculate last pair of cones
        last_orientation = curve_points[-1][2]
        last_right_x = curve_points[-1][0] - (self.track_width/2)*np.cos(last_orientation + np.pi/2)
        last_right_y = curve_points[-1][1] - (self.track_width/2)*np.sin(last_orientation + np.pi/2)
        last_left_x = curve_points[-1][0] + (self.track_width/2)*np.cos(last_orientation + np.pi/2)
        last_left_y = curve_points[-1][1] + (self.track_width/2)*np.sin(last_orientation + np.pi/2)
        self.cones.append((last_right_x, last_right_y, "yellow"))
        self.cones.append((last_left_x, last_left_y, "blue"))

        #calculate the first pair of cones
        first_orientation = curve_points[0][2]
        first_right_x = curve_points[0][0] - (self.track_width/2)*np.cos(first_orientation + np.pi/2)
        first_right_y = curve_points[0][1] - (self.track_width/2)*np.sin(first_orientation + np.pi/2)
        first_left_x = curve_points[0][0] + (self.track_width/2)*np.cos(first_orientation + np.pi/2)
        first_left_y = curve_points[0][1] + (self.track_width/2)*np.sin(first_orientation + np.pi/2)
        self.cones.append((first_right_x, first_right_y, "yellow"))
        self.cones.append((first_left_x, first_left_y, "blue"))

        distance_from_last_taken_cone = 0
        previous_point = curve_points[0]
        #calculate right cones
        for point in curve_points:
            x = point[0]
            y = point[1]
            orientation = point[2]
            curvature = point[3]

            #calculate distance from last cone
            distance = np.sqrt((x - previous_point[0])**2 + (y - previous_point[1])**2)
            distance_from_last_taken_cone += distance

            # calculate maximum distance between cones based on curvature
            max_cone_separation = (-26.92) * curvature + 5 # 5m in straight line, 1.5m in 9m radius curve

            # if the distance from the last cone is greater than the maximum cone separation or right_cone_separation, place a cone
            if (distance_from_last_taken_cone > max_cone_separation) or (distance_from_last_taken_cone > self.right_cone_separation):
                # right cone
                right_cone_x = x - (self.track_width/2)*np.cos(orientation + np.pi/2)
                right_cone_y = y - (self.track_width/2)*np.sin(orientation + np.pi/2)
                # add gaussian noise to the cone position
                right_cone_x += np.random.normal(0, self.std_deviation)
                right_cone_y += np.random.normal(0, self.std_deviation)

                #if its closer to the last cone than 0.5m, dont place it
                if np.sqrt((right_cone_x - last_right_x)**2 + (right_cone_y - last_right_y)**2) > 0.5:
                    self.cones.append((right_cone_x, right_cone_y, "yellow"))
                distance_from_last_taken_cone = 0
            
            previous_point = point

        #calculate left cones
        distance_from_last_taken_cone = 0
        previous_point = curve_points[0]
        for point in curve_points:
            x = point[0]
            y = point[1]
            orientation = point[2]
            curvature = point[3]

            #calculate distance from last cone
            distance = np.sqrt((x - previous_point[0])**2 + (y - previous_point[1])**2)
            distance_from_last_taken_cone += distance

            # calculate maximum distance between cones based on curvature
            max_cone_separation = (-26.92) * curvature + 5

            # if the distance from the last cone is greater than the maximum cone separation or right_cone_separation, place a cone
            if (distance_from_last_taken_cone > max_cone_separation) or (distance_from_last_taken_cone > self.left_cone_separation):
                # left cone
                left_cone_x = x + (self.track_width/2)*np.cos(orientation + np.pi/2)
                left_cone_y = y + (self.track_width/2)*np.sin(orientation + np.pi/2)
                # add gaussian noise to the cone position
                left_cone_x += np.random.normal(0, self.std_deviation)
                left_cone_y += np.random.normal(0, self.std_deviation)

                #if its closer to the last cone than 0.5m, dont place it
                if np.sqrt((left_cone_x - last_left_x)**2 + (left_cone_y - last_left_y)**2) > 0.5:
                    self.cones.append((left_cone_x, left_cone_y, "blue"))
                distance_from_last_taken_cone = 0
            
            previous_point = point

        return self.cones
    
    #plots the cones on the map
    def plot_map(self):
        for cone in self.cones:
            plt.plot(cone[0], cone[1], 'o', color = cone[2])
            plt.grid(True)
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.title("Map")
            plt.axis('equal')
            plt.legend(["Left Cone", "Right Cone"])
        plt.show()
