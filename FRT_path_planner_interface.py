import rospy
import path_planner_interface
import threading
import time
from frt_custom_msgs.msg import Map
from frt_custom_msgs.msg import ControlInfo
from frt_custom_msgs.msg import Landmark
from frt_custom_msgs.msg import TrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class FRT_path_planner_interface(path_planner_interface.path_planner_interface):
    def __init__(self):
        self.path = None
        self.path_received = threading.Event()
        self.path_received.clear()
        rospy.init_node('Tester', anonymous=True)
        self.pub = rospy.Publisher('/map', Map, queue_size=10)
        self.marker_pub = rospy.Publisher('/GA_visualization', MarkerArray, queue_size=10)
        rospy.Subscriber('/control_info', ControlInfo, self.control_info_callback)

        tries = 0
        # Wait until there is at least one subscriber
        tries = 0
        while self.pub.get_num_connections() < 1:
            if(tries > 5):
                print("No subscribers found")
                #stop execution
                exit()
            print("Waiting for subscribers")
            tries += 1
            rospy.sleep(0.5)  # Sleep for a short time to avoid busy waiting
            tries += 1

    def visualize_cones_and_path(self, cone_map, path):
        # Create a marker array to store the cones and the path
        marker_array = MarkerArray()

        # Visualize cones as colored cubes
        for i in range(len(cone_map)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.id = i + 1  # Unique ID for each marker
            marker.pose.position.x = cone_map[i][0]
            marker.pose.position.y = cone_map[i][1]
            marker.pose.position.z = 0.5  # Set the height of the cubes
            marker.scale.x = 0.4  # Set the size of the cubes
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            if cone_map[i][2] == "blue":
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            elif cone_map[i][2] == "yellow":
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
            marker.color.a = 1.0  # Set the alpha (transparency) value
            marker.lifetime = rospy.Duration(0.5)  # Persistent marker
            marker_array.markers.append(marker)

        # Visualize the path as a linestrip
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.id = 0  # Unique ID for the path marker
        path_marker.scale.x = 0.3  # Set the width of the line
        path_marker.color.r = 1.0
        path_marker.color.g = 1.0
        path_marker.color.b = 1.0
        path_marker.color.a = 1.0
        path_marker.lifetime = rospy.Duration(0)  # Persistent marker

        for point in path:
            path_marker.points.append(Point(x=point[0], y=point[1], z=0.0))

        marker_array.markers.append(path_marker)

        self.marker_pub.publish(marker_array)


    def control_info_callback(self, data):
        #clear path
        self.path = []

        # Store the path
        for i in range(len(data.trajectory)):
            point = data.trajectory[i]
            self.path.append((point.x, point.y))
        # Set the event to signal that the path has been received
        self.path_received.set()

    def visualize_cones_and_path(self, cone_map, path):
        marker_array = MarkerArray()

        # Visualize cones as colored cubes
        for i in range(len(cone_map)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.id = i + 1  # Unique ID for each marker
            marker.pose.position.x = cone_map[i][0]
            marker.pose.position.y = cone_map[i][1]
            marker.pose.position.z = 0.5  # Set the height of the cubes
            marker.scale.x = 0.4  # Set the size of the cubes
            marker.scale.y = 0.4
            marker.scale.z = 0.4

            if(cone_map[i][2] == "blue"):
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            elif(cone_map[i][2] == "yellow"):
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            
            marker.color.a = 1.0  # Set the alpha (transparency) value
            marker.lifetime = rospy.Duration(0.5)  # Persistent marker
            marker_array.markers.append(marker)

        # Visualize the path as a linestrip
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.id = 0  # Unique ID for the path marker
        path_marker.scale.x = 0.2  # Set the width of the line
        path_marker.color.r = 1.0
        path_marker.color.g = 1.0
        path_marker.color.b = 1.0
        path_marker.color.a = 1.0
        #path_marker.lifetime = rospy.Duration(0)  # Persistent marker

        for i in range(len(path)):
            path_marker.points.append(Point(x=path[i][0], y=path[i][1], z=0.0))

        marker_array.markers.append(path_marker)

        self.marker_pub.publish(marker_array)


    def calculate_path_on_map(self, cone_map):
        #create map msg
        map_msg = Map()
        id = 0
        #add landmarks to map msg
        for cone in cone_map:
            landmark = Landmark()
            landmark.x = cone[0]
            landmark.y = cone[1]
            if(cone[2] == "blue"):
                landmark.color = 1
            if(cone[2] == "yellow"):
                landmark.color = 2
            landmark.id = id
            id += 1
            map_msg.map.append(landmark)

        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"

        #publish map
        self.pub.publish(map_msg)
        #print("published map -> " + str(id))
        # Wait for the path to be received
        self.path_received.wait()
        # Reset the event for the next call
        self.path_received.clear()

        # visualize the path and map in rviz
        self.visualize_cones_and_path(cone_map, self.path)

        return self.path
    
