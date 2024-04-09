import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

class RelativePoseTracker:
    def __init__(self):
        # self.listener = tf.TransformListener()
        # self.broadcaster = tf.TransformBroadcaster()
        # Initialize self.relative_pose to identity
        self.relative_pose = np.array([[1, 0, 0, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]])
        self.pose_estimate = np.array([[1, 0, 0, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]])
        self.prev_pose = np.array([[1, 0, 0, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]])

    def update_pose(self):
        self.pose_estimate = np.dot(self.prev_pose, self.relative_pose)