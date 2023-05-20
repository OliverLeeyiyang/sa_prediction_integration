import rclpy
from rclpy.node import Node
import numpy as np

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



class SelfUtils():
    '''Methods for PathGenerator class, tested in TestClass.'''
    def __init__(self):
        print('SelfUtils class is ready!')


    # Methods:
    def calc_offset_pose(self, p: gmsgs.Pose, x: float, y: float, z: float) -> gmsgs.Pose:
        new_pose = gmsgs.Pose()
        transform = gmsgs.TransformStamped()
        transform.transform.translation = self.createTranslation(x, y, z)
        transform.transform.rotation = self.createQuaternion(0.0, 0.0, 0.0, 1.0)

        tf_offset = tf2_gmsgs.from_msg_msg(transform)
        tf_pose = tf2_gmsgs.from_msg_msg(p)
        new_pose = tf2_gmsgs.do_transform_pose(tf_pose, tf_offset)

        return new_pose


    def createTranslation(self, x: float, y: float, z: float) -> gmsgs.Vector3:
        v = gmsgs.Vector3()
        v.x = x
        v.y = y
        v.z = z

        return v


    def createQuaternion(self, x: float, y: float, z: float, w: float) -> gmsgs.Quaternion:
        q = gmsgs.Quaternion()
        q.x = x
        q.y = y
        q.z = z
        q.w = w

        return q



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



class TestClass():
    '''Test methods in SelfUtils class and ParellelPathGeneratorNode class'''

    def __init__(self):
        self.su = SelfUtils()
        self.ppgn = ParellelPathGeneratorNode(time_horizon, sampling_time_interval, min_crosswalk_user_velocity)


    def test_method_in_selfutils(self):
        self.x = 1.0
        self.y = 1.0
        self.z = 1.0
        self.pose = gmsgs.Pose()

        # Test methods in SelfUtils class
        print("Test methods in SelfUtils class:")

        # Test createQuaternion method
        print('Test createQuaternion method for (0.0, 0.0, 0.0, 1.0):')
        q = self.su.createQuaternion(0.0, 0.0, 0.0, 1.0)
        print('Quaternion is: ', q)

        # Test createTranslation method
        print('Test createTranslation method for (1.0, 1.0, 1.0):')
        v = self.su.createTranslation(self.x, self.y, self.z)
        print('Translation is: ', v)
        
        # Test calc_offset_pose method
        print('Test calc_offset_pose method for Pose (1.0, 1.0, 1.0) with Trans and Quat above:')
        self.pose.position.x = self.x
        self.pose.position.y = self.y
        self.pose.position.z = self.z
        self.pose.orientation = self.su.createQuaternion(0.0, 0.0, 0.0, 1.0)
        print('Original Pose is: ', self.pose)
        new_pose = self.su.calc_offset_pose(self.pose, self.x, self.y, self.z)
        print('New pose is: ', new_pose)
    

    def test_ppgn(self):
        pass



class ParellelPathGeneratorNode(Node):
    ''' Node for generating path for other vehicles and crosswalk users.
        Input topics:   /perception/object_recognition/tracking/objects
                        /map/vector_map
        Output topics:  /perception/object_recognition/objects'''

    def __init__(self, time_horizon, sampling_time_interval, min_crosswalk_user_velocity):
        super().__init__('parellel_path_generator_node')
        self.ppgn = PathGenerator(time_horizon, sampling_time_interval, min_crosswalk_user_velocity)

        self.object_sub = self.create_subscription(TrackedObjects, input_topic_objects, self.ppgn.object_callback, 10)
        self.pred_objects_pub = self.create_publisher(PredictedObjects, output_topic_objects, 10)



def main():
    rclpy.init()
    tc = TestClass()
    # tc.test_method_in_selfutils()

    # rclpy.spin(tc)
    # tc.destroy_node()
    # rclpy.shutdown()



if __name__ == '__main__':
    main()