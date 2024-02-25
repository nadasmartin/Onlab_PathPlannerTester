import bezier_curve
import map_generator

curve = bezier_curve.BezierCurve((0, 0), (0, 90), (70, 90), (70, 0))
curve.plot_curve(15)

map = map_generator.Map(curve)
map.plot_map()


