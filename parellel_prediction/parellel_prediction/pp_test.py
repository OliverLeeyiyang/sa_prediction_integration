from .parellel_path_generator import PathGenerator
from .map_based_prediction_node import ParellelPathGeneratorNode
from .from_tier4_utils import Tier4Utils

import rclpy
from rclpy.node import Node
import geometry_msgs.msg as gmsgs
import numpy as np
import tf_transformations 
import geopandas
from shapely.geometry import Point, LineString, MultiLineString
from array import array
import pickle

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
        self.tu = Tier4Utils()

        self.x = 1.0
        self.y = 1.0
        self.z = 1.0
        self.pose = gmsgs.Pose()

        # Test methods in SelfUtils class
        print("Test methods in SelfUtils class:")

        # Test createQuaternion method
        print('Test createQuaternion method for (0.0, 0.0, 0.0, 1.0):')
        q = self.tu.createQuaternion(0.0, 0.0, 0.0, 1.0)
        print('Quaternion is: ', q)

        # Test createTranslation method
        print('Test createTranslation method for (1.0, 1.0, 1.0):')
        v = self.tu.createTranslation(self.x, self.y, self.z)
        print('Translation is: ', v)
        
        # Test calc_offset_pose method
        print('Test calc_offset_pose method for Pose (1.0, 1.0, 1.0) with Trans and Quat above:')
        self.pose.position.x = self.x
        self.pose.position.y = self.y
        self.pose.position.z = self.z
        #self.pose.orientation = self.tu.createQuaternion(0.0, 0.0, 0.0, 1.0)
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        self.pose.orientation = self.tu.createQuaternion(quat[0], quat[1], quat[2], quat[3])
        print('Original Pose is: ', self.pose)
        new_pose = self.tu.calcoffsetpose(self.pose, self.x, self.y, self.z)
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
    

    def test_ppg(self):
        self.ppg = PathGenerator(time_horizon, sampling_time_interval, min_crosswalk_user_velocity)
        from typing import TypedDict
        FrenetPoint = TypedDict('FrenetPoint', {'s': float, 'd': float, 's_vel': float, 'd_vel': float, 's_acc': float, 'd_acc': float})
        cur = FrenetPoint()
        cur['s_vel'] = 1.0
        cur['d_vel'] = 1.0
        cur['d'] = 1.0
        cur['d_acc'] = 1.0
        tar = FrenetPoint()
        tar['s_vel'] = 2.0
        tar['d_vel'] = 2.0
        tar['d'] = 2.0
        tar['d_acc'] = 2.0
        print(self.ppg.calcLatCoefficients(cur, tar, 1))
    

    def test_geopandas(self):
        s = geopandas.GeoSeries([LineString([(0, 0), (1, 1), (0, 1)]),LineString([(10, 0), (10, 5), (0, 0)]),MultiLineString([((0, 0), (1, 0)), ((-1, 0), (1, 0))]),])
        print(s.length)
        print(s.length[0])
        print(s.length.max())
    
    def test_map(self):
        aa = array('l', [1, 2, 3, 4, 5])
        print('type aa is:', type(aa))
        ab = pickle.dumps(aa)
        print('ab is:', ab)
        ac = pickle.loads(ab)
        print('type ac is:', type(ac))
        print('ac is:', ac)\
        
    def test_time(self):
        from rclpy.time import Time
        time1 = self.get_clock().now()
        print('time1 is:', time1)
        time2 = time1.seconds_nanoseconds()
        print('time2 is:', time2)
        time3 = time2[0]
        print('time3 is:', time3)
        self.tu = Tier4Utils()
        time4 = self.tu.to_cpp_seconds(time2)
        print('time4 is:', time4)



def main(args=None):
    rclpy.init(args=args)

    tc = TestClass()
    print('Hi from pp_test.py')

    # tc.test_ppg()
    # tc.test_ran()
    
    #tc.test_method_in_selfutils()
    # tc.test_geopandas()
    # tc.test_map()
    tc.test_time()

    # rclpy.spin(tc)



if __name__ == '__main__':
    main()