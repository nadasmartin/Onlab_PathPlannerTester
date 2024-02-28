import rospy
import path_planner_interface
import threading
import time
from frt_custom_msgs.msg import Map
from frt_custom_msgs.msg import ControlInfo
from frt_custom_msgs.msg import Landmark
from frt_custom_msgs.msg import TrajectoryPoint

class FRT_path_planner_interface(path_planner_interface.path_planner_interface):
    def __init__(self):
        self.path = None
        self.path_received = threading.Event()
        self.path_received.clear()
        rospy.init_node('Tester', anonymous=True)
        self.pub = rospy.Publisher('/map', Map, queue_size=10)
        rospy.Subscriber('/control_info', ControlInfo, self.control_info_callback)
        # Wait until there is at least one subscriber
        while self.pub.get_num_connections() < 1:
            print("Waiting for subscribers")
            rospy.sleep(0.5)  # Sleep for a short time to avoid busy waiting

    def control_info_callback(self, data):
        #clear path
        self.path = []

        # Store the path
        for i in range(len(data.trajectory)):
            point = data.trajectory[i]
            self.path.append((point.x, point.y))
        # Set the event to signal that the path has been received
        self.path_received.set()

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
        print("published map -> " + str(id))
        # Wait for the path to be received
        self.path_received.wait()
        # Reset the event for the next call
        self.path_received.clear()
        return self.path
    
