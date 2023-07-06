# System and Projects imports
import rclpy
from rclpy.duration import Duration
import numpy as np
import geometry_msgs.msg as gmsgs
import tf_transformations
import math

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
from .from_tier4_utils import Tier4Utils

# New data types
from typing import List, Tuple, TypedDict
# Can use a class to setup this type. 
# Or use TypeDict <https://docs.python.org/3/library/typing.html#other-special-directives>
# FrenetPoint = TypedDict('FrenetPoint', {'s': float, 'd': float, 's_vel': float, 'd_vel': float, 's_acc': float, 'd_acc': float})
class FrenetPoint:
    def __init__(self, s: float, d: float, s_vel: float, d_vel: float, s_acc: float, d_acc: float):
        self.s = s
        self.d = d
        self.s_vel = s_vel
        self.d_vel = d_vel
        self.s_acc = s_acc
        self.d_acc = d_acc

FrenetPath = List[FrenetPoint]
Vector2d = Tuple[float, float]
EntryPoint = Tuple[Vector2d, Vector2d]
PosePath = List[gmsgs.Pose] # PosePath cannot be instantiated as a class, can be used as e.g. posepath1: PosePath = []


# F_dtype = [('s', np.float64), ('d', np.float64), ('s_vel', np.float64), ('d_vel', np.float64), ('s_acc', np.float64), ('d_acc', np.float64)]
# FrenetPath is a list of FrenetPoint
# FrenetPath = type('FrenetPath', (FrenePoint,), {'__repr__': lambda self: f'FrenetPath({self.FrenetPath})'})



class PathGenerator():
    ''' Genertate path for other vehicles and crosswalk users.

        Parameters: time_horizon, sampling_time_interval, min_crosswalk_user_velocity

        Output:     PredictedPath
    '''

    def __init__(self, time_horizon_, sampling_time_interval_, min_crosswalk_user_velocity_):
        self.tu = Tier4Utils()

        self.time_horizon = time_horizon_
        self.sampling_time_interval = sampling_time_interval_
        self.min_crosswalk_user_velocity = min_crosswalk_user_velocity_

        self.object = TrackedObject()


    def generatePathForNonVehicleObject(self, object: TrackedObject) -> PredictedPath:
        return self._generateStraightPath(object)
    

    # TODO: generatePathToTargetPoint, not sure about the input type of point.
    def generatePathToTargetPoint(self, object: TrackedObject, point: Vector2d) -> PredictedPath:
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
        return self._generateStraightPath(object)
    

    def generatePathForOnLaneVehicle(self, object: TrackedObject, ref_path: PosePath) -> PredictedPath:
        if len(ref_path) < 2:
            return self._generateStraightPath(object)
        else:
            return self._generatePolynomialPath(object, ref_path)


    def _generateStraightPath(self, object: TrackedObject) -> PredictedPath:
        object_pose = object.kinematics.pose_with_covariance.pose
        object_twist = object.kinematics.twist_with_covariance.twist
        ep = 0.001
        duration = self.time_horizon + ep

        path = PredictedPath()
        path.time_step = Duration.to_msg(Duration(seconds = self.sampling_time_interval))
        path.path = []

        dt = 0.0
        while dt < duration:
            future_obj_pose = self.tu.calcoffsetpose(object_pose, object_twist.linear.x * dt, object_twist.linear.y * dt, 0.0)
            path.path.append(future_obj_pose)
            dt += self.sampling_time_interval
        
        return path
    
    # TODO: generatePolynomialPath (line 178-208)
    def _generatePolynomialPath(self, object: TrackedObject, ref_path: PosePath) -> PredictedPath:
        pass

    # TODO: test
    def _generateFrenetPath(self, current_point: FrenetPoint, target_point: FrenetPoint, max_length: float) -> FrenetPath:
        path: FrenetPath() = []
        duration = self.time_horizon

        # Compute Lateral and Longitudinal Coefficients to generate the trajectory
        lat_coeff = self._calcLatCoefficients(current_point, target_point, duration)
        lon_coeff = self._calcLonCoefficients(current_point, target_point, duration)

        # C++ use reserve() to allocate memory for the vector, but python list is dynamic, no need to do this.
        # path.reserve(static_cast<size_t>(duration / sampling_time_interval_));
        t = 0.0
        while t <= duration:
            d_next = current_point.d + current_point.d_vel*t + 0*2*t**2 + lat_coeff[0][0]*t**3 + lat_coeff[1][0]*t**4 + lat_coeff[2][0]*t**5
            s_next = current_point.s + current_point.s_vel*t + 0*2*t**2 + lon_coeff[0][0]*t**3 + lon_coeff[1][0]*t**4
            if s_next > max_length:
                break

            # We assume the object is traveling at a constant speed along s direction
            point = FrenetPoint()
            point.s = np.maximum(s_next, 0.0)
            point.s_vel = current_point.s_vel
            point.s_acc = current_point.s_acc
            point.d = d_next
            point.d_vel = current_point.d_vel
            point.d_acc = current_point.d_acc
            path.append(point)

            t += self.sampling_time_interval
        
        return path



    # test passed
    def _calcLatCoefficients(self, current_point: FrenetPoint, target_point: FrenetPoint, T: float) -> np.ndarray:
        '''Lateral Path Calculation -> 3X1 vector
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
        A_lat_inv = np.matrix([[10/(T**3), -4/(T**2), 1/(2*T)],
                           [-15/(T**4), 7/(T**3), -1/(T**2)],
                           [6/(T**5), -3/(T**4),  1/(2*T**3)]])
        b_lat = np.matrix([[target_point.d - current_point.d - current_point.d_vel * T],
                        [target_point.d_vel - current_point.d_vel],
                        [target_point.d_acc]])
        result = A_lat_inv * b_lat

        return result

    # test passed
    def _calcLonCoefficients(self, current_point: FrenetPoint, target_point: FrenetPoint, T: float) -> np.ndarray:
        ''' Longitudinal Path Calculation
        -------------------------------
            Quadric polynomial
            A_inv = np.matrix(  [[1/(T**2),   -1/(3*T)],
                                [-1/(2*T**3), 1/(4*T**2)]])
            b = np.matrix( [[vxe - self.a1 - 2 * self.a2 * T],
                        [axe - 2 * self.a2]])

            output: 2X1 Vector
        -------------------------------
        '''
        A_lon_inv = np.matrix([[1/(T**2), -1/(3*T)],
                           [-1/(2*T**3), 1/(4*T**2)]])
        b_lon = np.matrix([[target_point.s_vel - current_point.s_vel],
                        [0.0]])
        result = A_lon_inv * b_lon

        return result


    def _interpolateReferencePath(self, base_path: PosePath, frenet_predicted_path: FrenetPath) -> PosePath:
        pass

    # TODO: test
    def _convertToPredictedPath(self, object: TrackedObject, frenet_predicted_path: FrenetPath, ref_path: PosePath) -> PredictedPath:
        predicted_path = PredictedPath()
        predicted_path.time_step = Duration.to_msg(Duration(seconds = self.sampling_time_interval))
        predicted_path.path = []
        for i in range(len(ref_path)):
            # Reference Point from interpolated reference path
            ref_pose = ref_path[i]
            # Frenet Point from frenet predicted path
            frenet_point = frenet_predicted_path[i]
            # Converted Pose
            predicted_pose = self.tu.calcoffsetpose(ref_pose, 0.0, frenet_point.d, 0.0)
            predicted_pose.position.z = object.kinematics.pose_with_covariance.pose.position.z

            if i == 0:
                predicted_pose.orientation = object.kinematics.pose_with_covariance.pose.orientation
            else:
                yaw = self.tu.calcAzimuthAngle(predicted_path.path[i-1].position, predicted_pose.position)
                predicted_pose.orientation = self.tu.createQuaternionFromYaw(yaw)
            
            predicted_path.path.append(predicted_pose)
        
        return predicted_path
    

    # TODO: test this method(TODO later) and the methods it calls(done)
    def _getFrenetPoint(self, object: TrackedObject, ref_path: PosePath) -> FrenetPoint:
        frenet_point = FrenetPoint()
        obj_point = object.kinematics.pose_with_covariance.pose.position

        nearest_segment_idx = self.tu.findNearestSegmentIndex(ref_path, obj_point)
        l = self.tu.calcLongitudinalOffsetToSegment(ref_path, nearest_segment_idx, obj_point)
        vx = object.kinematics.twist_with_covariance.twist.linear.x
        vy = object.kinematics.twist_with_covariance.twist.linear.y
        obj_yaw = self.tu.getYawFromQuaternion(object.kinematics.pose_with_covariance.pose.orientation)
        lane_yaw = self.tu.getYawFromQuaternion(ref_path[nearest_segment_idx].orientation)
        delta_yaw = obj_yaw - lane_yaw

        frenet_point.s = self.tu.calcSignedArcLength(ref_path, 0, nearest_segment_idx) + l
        frenet_point.d = self.tu.calcLateralOffset(ref_path, obj_point)
        frenet_point.s_vel = vx * np.cos(delta_yaw) - vy * np.sin(delta_yaw)
        frenet_point.d_vel = vx * np.sin(delta_yaw) + vy * np.cos(delta_yaw)
        frenet_point.s_acc = 0.0
        frenet_point.d_acc = 0.0



def main():
    pass



if __name__ == '__main__':
    main()