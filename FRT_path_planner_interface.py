import rospy
import path_planner_interface
import threading
from frt_custom_msgs.msg import Map

class FRT_path_planner_interface(path_planner_interface.path_planner_interface):
    def __init__(self):
        self.path = None
        self.path_received = threading.Event()
        rospy.init_node('path_planner', anonymous=True)
        self.pub = rospy.Publisher('/map', String, queue_size=10)
        rospy.Subscriber('/control_info', String, self.control_info_callback)

    def control_info_callback(self, data):
        self.path = data.data
        # Set the event to signal that the path has been received
        self.path_received.set()

    def calculate_path_on_map(self, map):
        self.pub.publish(map)
        # Wait for the path to be received
        self.path_received.wait()
        # Reset the event for the next call
        self.path_received.clear()
        return self.path