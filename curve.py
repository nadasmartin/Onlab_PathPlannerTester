
#this class is the interface for curves
class Curve:
    def __init__(self) -> None:
        pass

    #returns the length of the curve
    def get_curve_length(self, n_partitions = 1000):
        pass

    #returns the curve points
    def get_curve(self, n_partitions):
        pass