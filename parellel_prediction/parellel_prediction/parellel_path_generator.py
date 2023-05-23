# System and Projects imports
import rclpy
from rclpy.duration import Duration
import numpy as np
import geometry_msgs.msg as gmsgs

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

from typing import List, Tuple
# Here is still needed to be modified! 
# Consider using structured arrays in numpy. <https://numpy.org/doc/stable/user/basics.rec.html>
FrenePoint = List[float]
EntryPoint = Tuple[np.ndarray, np.ndarray]
FrenetPath = List[FrenePoint]
PosePath = List[gmsgs.Pose]



class PathGenerator():
    ''' Genertate path for other vehicles and crosswalk users.

        Parameters: time_horizon, sampling_time_interval, min_crosswalk_user_velocity

        Output:     PredictedPath
    '''

    def __init__(self, time_horizon_, sampling_time_interval_, min_crosswalk_user_velocity_):
        self.su = SelfUtils()

        self.time_horizon = time_horizon_
        self.sampling_time_interval = sampling_time_interval_
        self.min_crosswalk_user_velocity = min_crosswalk_user_velocity_

        self.object = TrackedObject()
        # self.ref_path = PosePath()


    def generatePathForNonVehicleObject(self, object: TrackedObject) -> PredictedPath:
        return self.generateStraightPath(object)
    

    # TODO: generatePathToTargetPoint, not sure about the input type of point.
    def generatePathToTargetPoint(self, object: TrackedObject, point: List[float]) -> PredictedPath:
        pass


    # TODO: generatePathForCrosswalkUser
    def generatePathForCrosswalkUser(self, object: TrackedObject, reachable_crosswalk: EntryPoint) -> PredictedPath:
        pass


    def generatePathForLowSpeedVehicle(self, object: TrackedObject) -> PredictedPath:
        path = PredictedPath()
        path.time_step = Duration.to_msg(Duration(seconds = self.sampling_time_interval))
        ep = 0.001
        duration = self.time_horizon + ep
        for dt in np.arange(0.0, duration, self.sampling_time_interval):
            path.path.append(object.kinematics.pose_with_covariance.pose)
        
        return path
    

    def generatePathForOffLaneVehicle(self, object: TrackedObject) -> PredictedPath:
        return self.generateStraightPath(object)
    

    def generatePathForOnLaneVehicle(self, object: TrackedObject, ref_path: np.ndarray) -> PredictedPath:
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
        path.time_step = Duration.to_msg(Duration(seconds = self.sampling_time_interval))
        path.path = []
        for dt in np.arange(0.0, duration, self.sampling_time_interval):
            future_obj_pose = self.su.calcoffsetpose(object_pose, object_twist.linear.x * dt, object_twist.linear.y * dt, 0.0)
            path.path.append(future_obj_pose)
        
        return path
    
    # TODO: generatePolynomialPath (line 178-208)
    def generatePolynomialPath(self, object: TrackedObject, ref_path: PosePath) -> PredictedPath:
        pass


    def generateFrenetPath(self, current_point: FrenePoint, target_point: FrenePoint, max_length: float) -> FrenetPath:
        pass


    def calcLatCoefficients(self, current_point: FrenePoint, target_point: FrenePoint, T: float) -> np.ndarray:
        '''Lateral Path Calculation
        -------------------------------
            Quintic polynomial for d

             A = np.array([[T**3, T**4, T**5],

                           [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],

                           [6 * T, 12 * T ** 2, 20 * T ** 3]])

             A_inv = np.matrix([[10/(T**3), -4/(T**2), 1/(2*T)],

                                [-15/(T**4), 7/(T**3), -1/(T**2)],

                                [6/(T**5), -3/(T**4),  1/(2*T**3)]])

             b = np.matrix([[xe - self.a0 - self.a1 * T - self.a2 * T**2],

                            [vxe - self.a1 - 2 * self.a2 * T],

                            [axe - 2 * self.a2]])
        -------------------------------
        '''
        pass


    def calcLonCoefficients(self, current_point: FrenePoint, target_point: FrenePoint, T: float):
        ''' Longitudinal Path Calculation
        -------------------------------
            Quadric polynomial
            A_inv = np.matrix(  [[1/(T**2),   -1/(3*T)],
                                [-1/(2*T**3), 1/(4*T**2)]])
            b = np.matrix( [[vxe - self.a1 - 2 * self.a2 * T],
                        [axe - 2 * self.a2]])
        -------------------------------
        '''
        pass


    def interpolateReferencePath(self, base_path: PosePath, frenet_predicted_path: FrenetPath) -> PosePath:
        pass


    def convertToPredictedPath(self, object: TrackedObject, frenet_predicted_path: FrenetPath, ref_path: PosePath) -> PredictedPath:
        pass


    def getFrenetPoint(self, object: TrackedObject, ref_path: PosePath) -> FrenePoint:
        pass



def main():
    pass



if __name__ == '__main__':
    main()