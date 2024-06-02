from matplotlib import patches
import numpy as np
import matplotlib.pyplot as plt
import cubic_spline_curve as cubic
import bezier_curve as bezier
class VisualizeConvergence():
    def __init__(self, filename):
        self.filename = filename
        self.data = np.genfromtxt(self.filename, delimiter=',', skip_header=1)
        
    def visulize(self, visu_bezier=False, visu_cubic=False):
        fig, ax = plt.subplots()
        ax.axhline(0, color='black')  # Add horizontal line y=0
        ax.axvline(0, color='black')  # Add vertical line x=0
        ax.scatter(self.data[:,4], self.data[:,3], label='point 2', color='red', s=10)
        ax.scatter(self.data[:,6], self.data[:,5],label='point 3', color='blue', s=10)
        full_zero = np.zeros(self.data[:,2].shape)
        ax.scatter(full_zero, self.data[:,2], label='point 1', color='green',s=10)

        # ax.axhline(0, color='black')  # Add horizontal line y=0
        # ax.axvline(0, color='black')  # Add vertical line x=0
        # ax.scatter(self.data[:,5], self.data[:,4], label='point 2', color='red', s=10)
        # ax.scatter(self.data[:,7], self.data[:,6],label='point 3', color='blue', s=10)
        # full_zero = np.zeros(self.data[:,2].shape)
        # ax.scatter(self.data[:,3], self.data[:,2], label='point 1', color='green',s=10)

        
        rect1 = patches.Rectangle((-6, 3), 12, 11, linewidth=1, edgecolor='r', facecolor='none')
        rect2 = patches.Rectangle((-12, +7.5), 24, 10, linewidth=1, edgecolor='b', facecolor='none')
        rect3 = patches.Rectangle((-0.05, +1.05), 0.1, 10, linewidth=1, edgecolor='g', facecolor='none')
        
        ax.add_patch(rect1)
        ax.add_patch(rect2)
        ax.add_patch(rect3)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Points')
        plt.legend()
        plt.axis('equal')
        plt.show()

        fig, ax = plt.subplots()
        ax.axhline(0, color='black')  # Add horizontal line y=0
        ax.axvline(0, color='black')  # Add vertical l
        # Draw lines from origin to each point
        for i in range(self.data.shape[0]):
            #print("p1x: ", self.data[i, 2], " p2x: ", self.data[i, 3], " p2y: ", self.data[i, 4], " p3x: ", self.data[i, 5], " p3y: ", self.data[i, 6])
            if visu_bezier:
                curve = bezier.BezierCurve((0, 0), (self.data[i, 2], 0), (self.data[i, 3], self.data[i, 4]), (self.data[i, 5], self.data[i, 6]))
            if visu_cubic:
                curve = cubic.CubicSplineCurve((0, 0), (self.data[i, 2], self.data[i, 3]), (self.data[i, 4], self.data[i, 5]), (self.data[i, 6], self.data[i, 7]))
            #curve.plot_curve()
            curve_points = np.array(curve.get_curve(20))
            #print("All x: ", curve_points[:,0], " all y: ", curve_points[:,1] )
            ax.plot(curve_points[:, 1], curve_points[:, 0], color='gray', alpha=0.5)
        
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('curves')
        plt.legend()
        plt.axis('equal')
        plt.show()



visu_conv = VisualizeConvergence("./logs/bezier_kibovitett.csv")
plot_cubic = False
plot_bezier = True
visu_conv.visulize(plot_bezier, plot_cubic)
