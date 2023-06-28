# System and Projects imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import io
import pickle

# Import message types
from autoware_auto_perception_msgs.msg import ObjectClassification
from autoware_auto_perception_msgs.msg import PredictedObject
from autoware_auto_perception_msgs.msg import PredictedObjectKinematics
from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_auto_perception_msgs.msg import PredictedPath
from autoware_auto_perception_msgs.msg import TrackedObject
from autoware_auto_perception_msgs.msg import TrackedObjectKinematics
from autoware_auto_perception_msgs.msg import TrackedObjects
from autoware_auto_perception_msgs.msg import DetectedObjectKinematics

import autoware_auto_mapping_msgs.msg as map_msgs
import geometry_msgs.msg as gmsgs
import std_msgs.msg as smsgs

# Outside imports
import math
from collections import deque # For objects_history
import lanelet2
import tf2_geometry_msgs as tf2_gmsgs
import tf_transformations 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from typing import List
from lanelet2.core import LaneletMap, ConstLanelet, Lanelet, registerId, LineString3d, Point3d, getId, createMapFromLanelets, AttributeMap
# print(dir(ConstLanelet.leftBound))
from lanelet2.routing import RoutingGraph
import lanelet2.traffic_rules as traffic_rules
import numpy as np

# Data structure
class LaneletData:
    def __init__(self):
        self.lanelet = lanelet2.core.Lanelet()
        self.probability: float
LaneletsData = List[LaneletData]

# Local imports
from .parellel_path_generator import PathGenerator
from .from_tier4_utils import Tier4Utils
from .map_loader import MapLoader

# Definition of new types
ConstLanelets = List[ConstLanelet]
Lanelets = List[Lanelet]

# input topics
input_topic_objects = '/perception/object_recognition/tracking/objects'
#input_topic_map = '/map/vector_map'
input_topic_map = '/output/lanelet2_map'

# output topics
output_topic_objects = '/perception/object_recognition/objects'
pareller_output_topic = '/parellel/objects'

# Parameters
time_horizon = 10.0
sampling_time_interval = 0.5
min_crosswalk_user_velocity = 1.0
# Maybe use Node.declare_parameter() to get parameters from launch file


# only for testing
import json



class ParellelPathGeneratorNode(Node):
    '''Node for generating path for other vehicles and crosswalk users.

    Launch simulation:

    cd ~/ma_prediction_integration/src/parellel_prediction/launch

    ros2 launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

    // To test map callback:

    ros2 run map_loader lanelet2_map_loader --ros-args -p lanelet2_map_path:=$HOME/autoware_map/sample-map-planning/lanelet2_map.osm

    ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

    Topics
    --------------------

    Input topics  :     /perception/object_recognition/tracking/objects : autoware_auto_perception_msgs/TrackedObjects
    
                        /map/vector_map : autoware_auto_mapping_msgs/msg/HADMapBin

    Output topics :     /perception/object_recognition/objects : autoware_auto_perception_msgs/PredictedObjects

                        /parellel/objects (Use this for testing)

    --------------------
    '''

    def __init__(self, time_horizon_, sampling_time_interval_, min_crosswalk_user_velocity_):
        super().__init__('parellel_path_generator_node')

        self.pg = PathGenerator(time_horizon_, sampling_time_interval_, min_crosswalk_user_velocity_)
        self.tu = Tier4Utils()

        self.map_sub = self.create_subscription(map_msgs.HADMapBin, input_topic_map, self.map_callback, 1)
        self.object_sub = self.create_subscription(TrackedObjects, input_topic_objects, self.object_callback, 10)
        self.pred_objects_pub = self.create_publisher(PredictedObjects, pareller_output_topic, 10)
        # Params for lanelet map
        self.lanelet_map = LaneletMap()
        self.all_lanelets = None
        self.traffic_rules = None
        self.routing_graph = None
        self.crosswalks_ = None

        self.tf_buffer = Buffer()
        self.objects_history_ = {}

        # test
        self.get_logger().info('[Parellel Map Based Prediction]: Start loading lanelet')
        map_file_path = '/home/oliver/autoware_map/sample-map-planning/lanelet2_map.osm'
        self.ml = MapLoader(map_file_path)
        self.lanelet_map = self.ml.load_map_for_prediction()
        self.get_logger().info('[Parellel Map Based Prediction]: Map is loaded')

    

    # line 152
    def updateLateralKinematicsVector(self):
        pass

    def map_callback(self, msg: map_msgs.HADMapBin):
        self.get_logger().info('[Parellel Map Based Prediction]: Start loading lanelet')
        self.fromBinMsg(msg)
        self.get_logger().info('[Parellel Map Based Prediction]: Map is loaded')
        
        self.all_lanelets = self.query_laneletLayer(self.lanelet_map)
        crosswalks = self.query_crosswalkLanelets(self.all_lanelets)
        walkways = self.query_walkwayLanelets(self.all_lanelets)
        self.crosswalks_ = crosswalks
        self.crosswalks_.extend(walkways)
        print('crosswalks: ', self.crosswalks_)


    def object_callback(self, in_objects: TrackedObjects):
        # Guard for map pointer and frame transformation
        # if self.lanelet_map is None:
        #     return
        # test
        print('[pyt called pose ]: ', in_objects[0].kinematics.pose_with_covariance.pose)
        print('[pyt called twist]: ', in_objects[0].kinematics.twist_with_covariance.twist)

        world2map_transform = self.tf_buffer.lookup_transform('map', in_objects.header.frame_id, in_objects.header.stamp, rclpy.duration.Duration(seconds=1.0))
        map2world_transform = self.tf_buffer.lookup_transform(in_objects.header.frame_id, 'map', in_objects.header.stamp, rclpy.duration.Duration(seconds=1.0))
        # debug_map2lidar_transform = self.tf_buffer.lookup_transform('base_link', 'map', Time(), rclpy.duration.Duration(seconds=1.0))

        if world2map_transform is None or map2world_transform is None:
            return
        #if world2map_transform is None or map2world_transform or
        #    return
        
        # Remove old objects information in object history(line 582) TODO: test this
        objects_detected_time = Time.from_msg(in_objects.header.stamp).seconds_nanoseconds()
        self.removeOldObjectsHistory(objects_detected_time)

        # result output
        output = PredictedObjects()
        output.header = in_objects.header
        output.header.frame_id = 'map'

        # Didn't do: debug_markers line 592

        # Deal with each object
        for object in in_objects.objects:
            # seems like this is not necessary
            object_id = self.tu.toHexString(object.object_id)

            transformed_object = object # transformed_object: TrackedObject()

            # transform object frame if it's based on map frame
            if in_objects.header.frame_id != 'map':
                pose_in_map = gmsgs.PoseStamped()
                pose_orig = gmsgs.PoseStamped()
                pose_orig.pose = object.kinematics.pose_with_covariance.pose
                pose_in_map = tf2_gmsgs.do_transform_pose(pose_orig, world2map_transform)
                transformed_object.kinematics.pose_with_covariance.pose = pose_in_map.pose

            # get tracking label and update it for the prediction
            tracking_label = transformed_object.classification[0].label
            label = self.changeLabelForPrediction(tracking_label)

            # TODO: For crosswalk user, don't consider this situation now.(line 612)
            if label == ObjectClassification.PEDESTRIAN or label == ObjectClassification.BICYCLE:
                predicted_object = self.getPredictedObjectAsCrosswalkUser(transformed_object)
                output.objects.append(predicted_object)

            # For road user
            elif label == ObjectClassification.CAR or label == ObjectClassification.BUS or label == ObjectClassification.TRAILER\
                    or label == ObjectClassification.MOTORCYCLE or label == ObjectClassification.TRUCK:
                # Update object yaw and velocity
                transformed_object = self.updateObjectData(transformed_object)
                # test
                print('[pyt updated pose ]: ', transformed_object.kinematics.pose_with_covariance.pose)
                print('[pyt updated twist]: ', transformed_object.kinematics.twist_with_covariance.twist)

                # TODO: Get Closest Lanelet (line 624)
                current_lanelets = self.getCurrentLanelets(transformed_object)

                # TODO: Update Objects History (line 627) **important for prediction**
                self.updateObjectsHistory(output.header, transformed_object, current_lanelets)

                # TODO: For off lane obstacles
                if current_lanelets is None:
                    predicted_path = self.pg.generatePathForOffLaneVehicle(transformed_object)
                    predicted_path.confidence = 1.0
                    if predicted_path.path is None:
                        continue

                    predicted_object = self.convertToPredictedObject(transformed_object)
                    predicted_object.kinematics.predicted_paths.append(predicted_path)
                    output.objects.append(predicted_object)
                    continue

                # TODO: For too-slow vehicle

                # Get Predicted Reference Path for Each Maneuver and current lanelets
                # return: <probability, paths>

                # If predicted reference path is empty, assume this object is out of the lane

                # Didn't do: Get Debug Marker for On Lane Vehicles

                # TODO: Generate Predicted Path

                # TODO: Normalize Path Confidence and output the predicted object

            # For unknown object
            else:
                predicted_object = self.convertToPredictedObject(transformed_object)
                predicted_path = self.pg.generatePathForNonVehicleObject(transformed_object)
                predicted_path.confidence = 1.0

                predicted_object.kinematics.predicted_paths.append(predicted_path)
                output.objects.append(predicted_object)
        
        # Publish results
        self.pred_objects_pub.publish(output)
    

    def convertToPredictedKinematics(self, tracked_object: TrackedObjectKinematics) -> PredictedObjectKinematics:
        output = PredictedObjectKinematics()
        output.initial_pose_with_covariance = tracked_object.pose_with_covariance
        output.initial_twist_with_covariance = tracked_object.twist_with_covariance
        output.initial_acceleration_with_covariance = tracked_object.acceleration_with_covariance

        return output
    

    def convertToPredictedObject(self, tracked_object: TrackedObject) -> PredictedObject:
        predicted_object = PredictedObject()
        predicted_object.kinematics = self.convertToPredictedKinematics(tracked_object.kinematics)
        predicted_object.classification = tracked_object.classification
        predicted_object.object_id = tracked_object.object_id
        predicted_object.shape = tracked_object.shape
        predicted_object.existence_probability = tracked_object.existence_probability

        return predicted_object


    def updateObjectData(self, object: TrackedObject) -> TrackedObject:
        if object.kinematics.orientation_availability == DetectedObjectKinematics.AVAILABLE:
            return
        
        # Compute yaw angle from the velocity and position of the object
        object_pose = object.kinematics.pose_with_covariance.pose
        object_twist = object.kinematics.twist_with_covariance.twist
        future_object_pose = self.tu.calcoffsetpose(object_pose, object_twist.linear.x * 0.1, object_twist.linear.y * 0.1, 0.0)

        if object.kinematics.twist_with_covariance.twist.linear.x < 0.0:
            if object.kinematics.orientation_availability == DetectedObjectKinematics.SIGN_UNKNOWN:
                original_yaw = self.tu.getYawFromQuaternion(object.kinematics.pose_with_covariance.pose.orientation)
                # flip the angle
                object.kinematics.pose_with_covariance.pose.orientation = self.tu.createQuaternionFromYaw(original_yaw + math.pi)
            else:
                update_object_yaw = self.tu.calcAzimuthAngle(object_pose.position, future_object_pose.position)
                object.kinematics.pose_with_covariance.pose.orientation = self.tu.createQuaternionFromYaw(update_object_yaw)
            
            object.kinematics.twist_with_covariance.twist.linear.x *= -1.0
        
        return object
    

    def removeOldObjectsHistory(self, current_time: Time):
        invalid_object_id = []
        for object_id, object_data in self.objects_history_.items():
            # If object data is empty, we are going to delete the buffer for the obstacle
            if not object_data:
                invalid_object_id.append(object_id)
                continue

            latest_object_time = object_data[-1].header.stamp.to_msg().seconds_nanoseconds()

            # Delete Old Objects
            if current_time - latest_object_time > 2.0:
                invalid_object_id.append(object_id)
                continue

            # Delete old information
            while object_data and current_time - object_data[0].header.stamp.to_msg().seconds_nanoseconds() > 2.0:
                # Delete Old Position
                object_data.popleft()

            if not object_data:
                invalid_object_id.append(object_id)
                continue

        for key in invalid_object_id:
            del self.objects_history_[key]


    # TODO: line 1049
    def updateObjectsHistory(self, header: smsgs.Header, object: TrackedObject, current_lanelets_data: LaneletsData):
        pass


    # TODO: finish this method for case 2 and 3. And input lanelet_map_ptr should be lanelet::LaneletMapPtr
    # still have two inputs: object: TrackedObject, lanlet_map_ptr: LaneletMapPtr
    def changeLabelForPrediction(self, label: ObjectClassification.label) -> ObjectClassification.label:
        ''' Change label for prediction.

            Case 1: For CAR, BUS, TRUCK, TRAILER, UNKNOWN, do not change label.

            Case 2: For BICYCLE and MOTORCYCLE(unimplemented yet)

            Case 3: For PEDESTRIAN(unimplemented yet)

        '''
        # for car like vehicle do not change labels
        if label == ObjectClassification.CAR or label == ObjectClassification.BUS\
             or label == ObjectClassification.TRUCK or label == ObjectClassification.TRAILER or label == ObjectClassification.UNKNOWN:
            return label
        # for bicycle and motorcycle
        elif label == ObjectClassification.MOTORCYCLE or label == ObjectClassification.BICYCLE:
            # TODO: change laber setting(line 440)
            return label
        # for pedestrian
        elif label == ObjectClassification.PEDESTRIAN:
            return label
        # TODO: new method in this class withinRoadLanelet for case 2 and 3(line 261)

    
    # TODO: getPredictedObjectAsCrosswalkUser
    def getPredictedObjectAsCrosswalkUser(self, object: TrackedObject) -> PredictedObject:
        ''' Get predicted object as crosswalk user.

        '''
        pass


    # Following methods are for lanelet2 utils
    def fromBinMsg(self, msg: map_msgs.HADMapBin):
        if self.lanelet_map is None:
            print("map is null pointer!")
            return
        
        # Method1
        data_str = str(msg.data)
        print('data_str: ', data_str[-1000:])
        # data_stream = io.BytesIO(data_str)
        # n = pickle.load(data_stream)
        # print('numbers: ', n)
        
        map = pickle.loads(data_str)
        id = pickle.loads(data_str)
        print(id)
        # Method2
        # data_str = [str(num) for num in msg.data.tolist()]
        data_list = msg.data.tolist()
        for d in data_list:
            a_lanelet = self.get_a_lanelet(d)
            self.lanelet_map.add(a_lanelet)
        # self.lanelet_map = createMapFromLanelets(self.get_a_lanelet(d) for d in data_str)

        id_counter = np.array(data_list, dtype=np.int64).tolist() # id_counter: lanelet2.Id, type: int64
        for id in id_counter:
            registerId(id)
        self.get_logger().info('[Parellel Map Based Prediction]: Id is registered!')
        
        self.traffic_rules = traffic_rules.create(traffic_rules.Locations.Germany, traffic_rules.Participants.Vehicle)
        # self.routing_graph = RoutingGraph(self.lanelet_map, self.traffic_rules)
        self.get_logger().info('[Parellel Map Based Prediction]: Routing graph is created!')
    

    def get_linestring_at_y(self, y):
        return LineString3d(getId(), [Point3d(getId(), i, y, 0) for i in range(0, 3)])
    

    def get_a_lanelet(self, index=0):
        return Lanelet(getId(),
                        self.get_linestring_at_y(2+index),
                        self.get_linestring_at_y(0+index))


    def query_subtypeLanelets(self, lls: ConstLanelets, subtype) -> ConstLanelets:
        subtype_lanelets: ConstLanelets() = []
        for ll in lls:
            print(ll.attributes)
            #if ll.attributes["Subtype"] == subtype:
            #   subtype_lanelets.append(ll)
        
        return subtype_lanelets


    def query_laneletLayer(self, ll_map: LaneletMap) -> ConstLanelets:
        lanelets: ConstLanelets() = []
        if ll_map is None:
            print("No map received!")
            return lanelets
        
        for ll in ll_map.laneletLayer:
            lanelets.append(ll)
        
        return lanelets
    

    def query_crosswalkLanelets(self, lls: ConstLanelets) -> ConstLanelets:
        return self.query_subtypeLanelets(lls, "crosswalk")


    def query_walkwayLanelets(self, lls: ConstLanelets) -> ConstLanelets:
        return self.query_subtypeLanelets(lls, "walkway")
    

    def getCurrentLanelets(self, object: TrackedObject) -> LaneletsData:
        return None


def main(args=None):
    rclpy.init(args=args)

    ppgn = ParellelPathGeneratorNode(time_horizon, sampling_time_interval, min_crosswalk_user_velocity)

    rclpy.spin(ppgn)



if __name__ == '__main__':
    main()