# System and Projects imports
import rclpy
from rclpy.duration import Duration
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Twist

# Autoware auto msgs
import autoware_auto_perception_msgs.msg._predicted_objects as apmsg_pos
import autoware_auto_perception_msgs.msg._tracked_objects as apmsg_tos

from autoware_auto_perception_msgs.msg import ObjectClassification
from autoware_auto_perception_msgs.msg import PredictedObject
from autoware_auto_perception_msgs.msg import PredictedObjectKinematics
from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_auto_perception_msgs.msg import PredictedPath
from autoware_auto_perception_msgs.msg import TrackedObject
from autoware_auto_perception_msgs.msg import TrackedObjectKinematics
from autoware_auto_perception_msgs.msg import TrackedObjects

# Local imports
from .self_utils import SelfUtils

""" from typing import List, Tuple
# Here is still needed to be modified!
EntryPoint = Tuple[np.ndarray, np.ndarray]
FrenetPath = List[np.ndarray]
PosePath = List[Pose] """



class PathGenerator():
    ''' Genertate path for other vehicles and crosswalk users
        Input:      TrackedObjects, PosePath
        Parameters: time_horizon, sampling_time_interval, min_crosswalk_user_velocity
        Output:     PredictedPath'''

    def __init__(self, time_horizon_, sampling_time_interval_, min_crosswalk_user_velocity_):
        self.su = SelfUtils()

        self.time_horizon = time_horizon_
        self.sampling_time_interval = sampling_time_interval_
        self.min_crosswalk_user_velocity = min_crosswalk_user_velocity_

        self.object = TrackedObject()
        # self.ref_path = PosePath()


    def generatePathForNonVehicleObject(self, object: TrackedObject) -> PredictedPath:
        return self.generateStraightPath(object)
    

    def generatePathForOffLaneVehicle(self, object: TrackedObject) -> PredictedPath:
        return self.generateStraightPath(object)
    
    """ 
    def generatePathForOnLaneVehicle(self, object: TrackedObject, ref_path: PosePath) -> PredictedPath:
        if len(ref_path) < 2:
            return self.generateStraightPath(object)
        else:
            return self.generatePolynomialPath(object, ref_path)
    """

    def generateStraightPath(self, object: TrackedObject) -> PredictedPath:
        object_pose = object.kinematics.pose_with_covariance.pose
        object_twist = object.kinematics.twist_with_covariance.twist
        ep = 0.001
        duration = self.time_horizon + ep

        path = PredictedPath()
        path.time_step = Duration.to_msg(Duration(seconds = self.sampling_time_interval))
        path.path = []
        for dt in np.arange(0.0, duration, self.sampling_time_interval):
            future_obj_pose = self.su.calcoffsetpose(object_pose, object_twist.linear.x * dt, object_twist.linear.y * dt, 0.0)
            path.path.append(future_obj_pose)
        
        return path
    
    '''
    def generatePolynomialPath(self, object: TrackedObject, ref_path: PosePath) -> PredictedPath:
        pass
    '''



def main():
    pass



if __name__ == '__main__':
    main()