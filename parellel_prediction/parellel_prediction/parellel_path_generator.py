import rclpy
from rclpy.node import Node
import numpy as np
from .self_utils import SelfUtils

# Autoware auto msgs
import autoware_auto_perception_msgs.msg._predicted_objects as apmsg_pos
import autoware_auto_perception_msgs.msg._tracked_objects as apmsg_tos

from geometry_msgs.msg import Pose, PoseStamped, Twist
import tf2_geometry_msgs as tf2_gmsgs
import geometry_msgs.msg as gmsgs

# namespeces in cpp file
from autoware_auto_perception_msgs.msg import ObjectClassification
from autoware_auto_perception_msgs.msg import PredictedObject
from autoware_auto_perception_msgs.msg import PredictedObjectKinematics
from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_auto_perception_msgs.msg import PredictedPath
from autoware_auto_perception_msgs.msg import TrackedObject
from autoware_auto_perception_msgs.msg import TrackedObjectKinematics
from autoware_auto_perception_msgs.msg import TrackedObjects

from typing import List, Tuple
# Here are still to be modified!
EntryPoint = Tuple[np.ndarray, np.ndarray]
FrenetPath = List[np.ndarray]
PosePath = List[Pose]

# input topics
input_topic_objects = '/perception/object_recognition/tracking/objects'
input_topic_map = '/map/vector_map'

# output topics
output_topic_objects = '/perception/object_recognition/objects'

# Parameters
time_horizon = 10.0
sampling_time_interval = 0.1
min_crosswalk_user_velocity = 0.1



class PathGenerator():
    ''' Genertate path for other vehicles and crosswalk users
        Input:      TrackedObjects, PosePath
        Parameters: time_horizon, sampling_time_interval, min_crosswalk_user_velocity
        Output:     PredictedPath'''

    def __init__(self, time_horizon, sampling_time_interval, min_crosswalk_user_velocity):
        self.su = SelfUtils()

        self.time_horizon = time_horizon
        self.sampling_time_interval = sampling_time_interval
        self.min_crosswalk_user_velocity = min_crosswalk_user_velocity

        self.object = TrackedObject()
        self.ref_path = PosePath()


    def object_callback(self, msg: TrackedObjects):
        self.object = msg.objects[0]

        #test
        self.test_path = self.generateStraightPath(self.object)
        self.test_path_pub.publish(self.test_path)
    

    def generatePathForNonVehicleObject(self, object: TrackedObject) -> PredictedPath:
        return self.generateStraightPath(object)
    

    def generatePathForOffLaneVehicle(self, object: TrackedObject) -> PredictedPath:
        return self.generateStraightPath(object)
    

    def generatePathForOnLaneVehicle(self, object: TrackedObject, ref_path: PosePath) -> PredictedPath:
        if len(ref_path) < 2:
            return self.generateStraightPath(object)
        else:
            return self.generatePolynomialPath(object, ref_path)


    def generateStraightPath(self, object: TrackedObject) -> PredictedPath:
        object_pose = object.kinematics.pose_with_covariance.pose
        object_twist = object.kinematics.twist_with_covariance.twist
        ep = 0.001
        duration = self.time_horizon + ep

        path = PredictedPath()
        path.time_step = rclpy.Duration.from_seconds(self.sampling_time_interval)
        path.path = []
        for dt in np.range(0.0, duration, self.sampling_time_interval):
            future_obj_pose = self.su.calc_offset_pose(object_pose, object_twist.linear.x * dt, object_twist.linear.y * dt, 0.0)
            path.path.append(future_obj_pose)
        
        return path
    

    def generatePolynomialPath(self, object: TrackedObject, ref_path: PosePath) -> PredictedPath:
        pass



def main():
    pass


if __name__ == '__main__':
    main()