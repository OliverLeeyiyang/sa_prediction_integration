from .parellel_path_generator import PathGenerator
from .map_based_prediction_node import ParellelPathGeneratorNode
from .self_utils import SelfUtils
from .from_tier4_utils import Tier4Utils

import rclpy
from rclpy.node import Node
import geometry_msgs.msg as gmsgs

# input topics
input_topic_objects = '/perception/object_recognition/tracking/objects'
input_topic_map = '/map/vector_map'

# output topics
output_topic_objects = '/perception/object_recognition/objects'

# Parameters
time_horizon = 10.0
sampling_time_interval = 0.5
min_crosswalk_user_velocity = 1.0



class TestClass(Node):
    '''Test methods in SelfUtils class and ParellelPathGeneratorNode class'''

    def __init__(self):
        super().__init__('test_class_node')


    def test_method_in_selfutils(self):
        self.su = SelfUtils()

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
        new_pose = self.su.calcoffsetpose(self.pose, self.x, self.y, self.z)
        print('New pose is: ', new_pose)
    

    def test_ppgn(self):
        ''' Test instructions:

        To test this method, we need to run:
        --------------------

        $ ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
        
        $ ros2 topic echo /perception/object_recognition/tracking/objects
        
        $ ros2 topic echo /test_pred_path
        
        $ ros2 topic echo /perception/object_recognition/objects

        --------------------
        '''
        self.ppgn = ParellelPathGeneratorNode(time_horizon, sampling_time_interval, min_crosswalk_user_velocity)
        print('ParellelPathGeneratorNode class is ready!')
    

    def test_ran(self):
        self.tu = Tier4Utils()
        point1 = gmsgs.Point()
        point1.x = 1.0
        point1.y = 1.0
        point1.z = 1.0
        point2 = gmsgs.Point()
        point2.x = 2.0
        point2.y = 2.0
        point2.z = 2.0
        point3 = gmsgs.Point()
        point3.x = 3.0
        point3.y = 3.0
        point3.z = 3.0

        from typing import List
        PosePath = List[gmsgs.Pose]
        points: PosePath = []
        pose1 = gmsgs.Pose()
        pose1.position = point1
        points.append(pose1)
        pose2 = gmsgs.Pose()
        pose2.position = point2
        points.append(pose2)
        pose3 = gmsgs.Pose()
        pose3.position = point3
        points.append(pose3)
        # print('idx is:', self.tu.findNearestSegmentIndex(points, point3))
        # print('list is:', self.tu.removeOverlapPoints(points))
        # print('dist is:', self.tu.calcLateralOffset(points, point3))
        # print('dist is:', self.tu.calcSignedArcLength(points, 0,2))
        # print('dist is:', self.tu.calcLongitudinalOffsetToSegment(points,1, point3))


def main(args=None):
    rclpy.init(args=args)

    tc = TestClass()
    print('Hi from pp_test.py')
    # tc.test_ran()
    
    # tc.test_method_in_selfutils()
    # tc.test_ppgn()
    # if tc.ppgn.flag:
    #     print('Path generated!.')

    # rclpy.spin(tc)



if __name__ == '__main__':
    main()