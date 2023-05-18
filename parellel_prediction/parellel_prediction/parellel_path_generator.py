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
EntryPoint = Tuple[np.ndarray, np.ndarray]
FrenetPath = List[np.ndarray]
PosePath = List[Pose]

# input topics
input_topic_objects = '/perception/object_recognition/tracking/objects'



# Methods are all in class SelfUtils
class SelfUtils():
    def __init__(self):
        print('SelfUtils class is created')

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


# Main class for parellel path generation
class PathGenerator():

    def __init__(self, time_horizon, sampling_time_interval, min_crosswalk_user_velocity):
        self.su = SelfUtils()

        self.time_horizon = time_horizon
        self.sampling_time_interval = sampling_time_interval
        self.min_crosswalk_user_velocity = min_crosswalk_user_velocity

        self.object = TrackedObject()
        self.ref_path = PosePath()
        self.object_sub = self.create_subscription(TrackedObjects, input_topic_objects, self.object_callback, 10)
        
        #Test
        self.test_path = PredictedPath()
        self.test_path_pub = self.create_publisher(PredictedPath, '/test_pub', 10)

    def object_callback(self, msg: TrackedObjects):
        self.object = msg.objects[0]

        #test
        self.test_path = self.generateStraightPath(self.object)
        self.test_path_pub.publish(self.test_path)


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


# This class is for testing methods
class TestClass():

    def __init__(self):
        self.su = SelfUtils()

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


class ParellelPathGeneratorNode(Node):

    def __init__(self):
        super().__init__('parellel_path_generator_node')


def main():
    rclpy.init()
    tc = TestClass()
    tc.test_method_in_selfutils()


if __name__ == '__main__':
    main()