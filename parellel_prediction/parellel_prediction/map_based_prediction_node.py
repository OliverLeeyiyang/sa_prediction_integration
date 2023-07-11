# System and Projects imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time

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
from lanelet2.core import LaneletMap, ConstLanelet, Lanelet, registerId, LineString3d, Point3d, getId, BasicPoint2d, ConstLineString2d

from lanelet2.routing import RoutingGraph, RelationType
import lanelet2.traffic_rules as traffic_rules
import lanelet2.geometry as l2_geom
import numpy as np

# Data structure
class LaneletData:
    def __init__(self):
        self.lanelet: Lanelet()
        self.probability: float

LaneletsData = List[LaneletData]

class LateralKinematicsToLanelet:
    def __init__(self):
        self.dist_from_left_boundary = 0.0
        self.dist_from_right_boundary = 0.0
        self.left_lateral_velocity = 0.0
        self.right_lateral_velocity = 0.0
        self.filtered_left_lateral_velocity = 0.0
        self.filtered_right_lateral_velocity = 0.0

class ObjectData:
    def __init__(self):
        self.header: smsgs.Header
        self.current_lanelets: ConstLanelets
        self.future_possible_lanelets: ConstLanelets
        self.pose: gmsgs.Pose
        self.twist: gmsgs.Twist
        self.time_delay: float
        self.lateral_kinematics_set = {} # key: ConstLanelet, value: LateralKinematicsToLanelet

# Local imports
from .parellel_path_generator import PathGenerator
from .from_tier4_utils import Tier4Utils
from .map_loader import MapLoader

# Definition of new types
ConstLanelets = List[ConstLanelet]
Lanelets = List[Lanelet]

# input topics
input_topic_objects = '/perception/object_recognition/tracking/objects'
# Here we don't subscribe the map topic, instead, we use map_loader to load map
# input_topic_map = '/map/vector_map'
# input_topic_map = '/output/lanelet2_map'

# output topics
output_topic_objects = '/perception/object_recognition/objects'
pareller_output_topic = '/parellel/objects'

# Parameters
time_horizon = 10.0
sampling_time_interval = 0.5
min_crosswalk_user_velocity = 1.0

# Not used so far
enable_delay_compensation_ = True
prediction_time_horizon_ = 10.0
prediction_sampling_time_interval_ = 0.5
min_velocity_for_map_based_prediction_ = 1.0
min_crosswalk_user_velocity_ = 1.0
object_buffer_time_length_ = 2.0
history_time_length_ = 1.0
dist_threshold_to_bound_ = 1.0  # 1.0m
time_threshold_to_bound_ = 5.0  # 5.0s
reference_path_resolution_ = 0.5

# Used parameters
delta_yaw_threshold_for_searching_lanelet_ = 0.785
dist_threshold_for_searching_lanelet_ = 3.0
sigma_lateral_offset_ = 0.5
sigma_yaw_angle_deg_ = 5.0
cutoff_freq_of_velocity_lpf_ = 0.1 # 0.1Hz
# Maybe use Node.declare_parameter() to get parameters from launch file


# load params from yaml file
Map_Path = '/home/oliver/ma_prediction_integration/src/maps/sample-map-planning-modi/lanelet2_map.osm'




class ParellelPathGeneratorNode(Node):
    '''Node for generating path for other vehicles and crosswalk users.

    // To launch self-modified simulation:

    cd ~/ma_prediction_integration/src/parellel_prediction/launch

    ros2 launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

    // To test map callback:

    ros2 run map_loader lanelet2_map_loader --ros-args -p lanelet2_map_path:=$HOME/autoware_map/sample-map-planning/lanelet2_map.osm

    ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

    // Use this for final testing!
    ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/ma_prediction_integration/src/maps/sample-map-planning-modi vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

    ros2 run parellel_prediction parellel_map_based_prediction_node

    ros2 run parellel_prediction visual_node

    Topics
    --------------------

    Input topics  :     /perception/object_recognition/tracking/objects : autoware_auto_perception_msgs/TrackedObjects
    
                        /map/vector_map : autoware_auto_mapping_msgs/msg/HADMapBin (Not used)

    Output topics :     /perception/object_recognition/objects : autoware_auto_perception_msgs/PredictedObjects

                        /parellel/objects (Use this for testing)

    --------------------
    '''

    def __init__(self, time_horizon_, sampling_time_interval_, min_crosswalk_user_velocity_):
        super().__init__('parellel_path_generator_node')

        self.pg = PathGenerator(time_horizon_, sampling_time_interval_, min_crosswalk_user_velocity_)
        self.tu = Tier4Utils()

        # self.map_sub = self.create_subscription(map_msgs.HADMapBin, input_topic_map, self.map_callback, 1)
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

        # load lanelet map for prediction
        self.get_logger().info('[Parellel Map Based Prediction]: Start loading lanelet')
        map_file_path = Map_Path
        self.ml = MapLoader(map_file_path)
        self.lanelet_map = self.ml.load_map_for_prediction()
        self.get_logger().info('[Parellel Map Based Prediction]: Map is loaded')

        self.all_lanelets = self.query_laneletLayer(self.lanelet_map) # also rigister Id in this method
        self.get_logger().info('[Parellel Map Based Prediction]: Id is registered!')
        crosswalks = self.query_crosswalkLanelets(self.all_lanelets)
        walkways = self.query_walkwayLanelets(self.all_lanelets)
        self.crosswalks_ = crosswalks
        self.crosswalks_.extend(walkways)
        # print('crosswalks: ', self.crosswalks_)
        
        self.traffic_rules = traffic_rules.create(traffic_rules.Locations.Germany, traffic_rules.Participants.Vehicle)
        self.routing_graph = RoutingGraph(self.lanelet_map, self.traffic_rules)
        self.get_logger().info('[Parellel Map Based Prediction]: Routing graph is created!')

    

    # This methods won't be used because we can't use boost in python
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
        if self.lanelet_map is None:
            return
        # test
        # print('[pyt called pose ]: ', in_objects[0].kinematics.pose_with_covariance.pose)
        # print('[pyt called twist]: ', in_objects[0].kinematics.twist_with_covariance.twist)

        world2map_transform = self.tf_buffer.lookup_transform('map', in_objects.header.frame_id, in_objects.header.stamp, rclpy.duration.Duration(seconds=1.0))
        map2world_transform = self.tf_buffer.lookup_transform(in_objects.header.frame_id, 'map', in_objects.header.stamp, rclpy.duration.Duration(seconds=1.0))
        # debug_map2lidar_transform = self.tf_buffer.lookup_transform('base_link', 'map', Time(), rclpy.duration.Duration(seconds=1.0))

        if world2map_transform is None or map2world_transform is None:
            return
        
        # Remove old objects information in object history(line 582) TODO: test this
        objects_detected_time = self.tu.to_cpp_seconds(Time.from_msg(in_objects.header.stamp).seconds_nanoseconds())
        self.removeOldObjectsHistory(objects_detected_time)

        # result output
        output = PredictedObjects()
        output.header = in_objects.header
        output.header.frame_id = 'map'

        # Didn't do: debug_markers line 592

        for object in in_objects.objects:
            # seems like this is not necessary
            object_id = self.tu.toHexString(object.object_id)

            transformed_object: TrackedObject = object

            # transform object frame if it's based on map frame
            if in_objects.header.frame_id != 'map':
                pose_in_map = gmsgs.PoseStamped()
                pose_orig = gmsgs.PoseStamped()
                pose_orig.pose = object.kinematics.pose_with_covariance.pose
                pose_in_map = tf2_gmsgs.do_transform_pose(pose_orig, world2map_transform)
                transformed_object.kinematics.pose_with_covariance.pose = pose_in_map.pose

            # get tracking label and update it for the prediction
            tracking_label = transformed_object.classification[0].label
            label = self.changeLabelForPrediction(tracking_label, object, self.lanelet_map)
            # self.get_logger().info('[Parellel Map Based Prediction]: label for prediction: ' + str(label))

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
                # print('[pyt updated pose ]: ', transformed_object.kinematics.pose_with_covariance.pose)
                # print('[pyt updated twist]: ', transformed_object.kinematics.twist_with_covariance.twist)

                # TODO: Get Closest Lanelet (line 624)
                current_lanelets = self.getCurrentLanelets(transformed_object)
                # if len(current_lanelets) > 0:
                #     print('current_lanelets: ', current_lanelets[0].lanelet)

                # TODO: Update Objects History (line 627) **important for prediction**
                self.updateObjectsHistory(output.header, transformed_object, current_lanelets)

                # TODO: For off lane obstacles
                if len(current_lanelets) == 0:
                    # test
                    print('off lane object detected')
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
    

    def removeOldObjectsHistory(self, current_time: float):
        invalid_object_id = []
        for object_id, object_data in self.objects_history_.items():
            # If object data is empty, we are going to delete the buffer for the obstacle
            if not object_data:
                invalid_object_id.append(object_id)
                continue

            latest_object_time = self.tu.to_cpp_seconds(Time.from_msg(object_data[-1].header.stamp).seconds_nanoseconds())

            # Delete Old Objects
            if current_time - latest_object_time > 2.0:
                invalid_object_id.append(object_id)
                continue

            # Delete old information
            while object_data:
                post_object_time = self.tu.to_cpp_seconds(Time.from_msg(object_data[0].header.stamp).seconds_nanoseconds())
                if current_time - post_object_time > 2.0:
                    # Delete Old Position
                    del object_data[0]
                else:
                    break

            if not object_data:
                invalid_object_id.append(object_id)
                continue

        for key in invalid_object_id:
            del self.objects_history_[key]


    # TODO: test
    def updateObjectsHistory(self, header: smsgs.Header, object: TrackedObject, current_lanelets_data: LaneletsData):
        object_id = self.tu.toHexString(object.object_id)
        current_lanelets = self.getLanelets(current_lanelets_data)
        # test
        # print("current_lanelets updated: ", current_lanelets)

        single_object_data = ObjectData()
        single_object_data.header = header
        single_object_data.current_lanelets = current_lanelets
        single_object_data.future_possible_lanelets = current_lanelets
        single_object_data.pose = object.kinematics.pose_with_covariance.pose

        object_yaw = self.tu.getYawFromQuaternion(object.kinematics.pose_with_covariance.pose.orientation)
        single_object_data.pose.orientation = self.tu.createQuaternionFromYaw(object_yaw)
        # same as cpp file in line 1063
        time_now_in_seconds = self.tu.to_cpp_seconds(self.get_clock().now().seconds_nanoseconds())
        time_header_in_seconds = self.tu.to_cpp_seconds(Time.from_msg(header.stamp).seconds_nanoseconds())
        single_object_data.time_delay = abs(time_now_in_seconds - time_header_in_seconds)

        single_object_data.twist = object.kinematics.twist_with_covariance.twist

        # Init lateral kinematics
        for current_lane in current_lanelets:
            lateral_kinematics: LateralKinematicsToLanelet = self.initLateralKinematics(current_lane, single_object_data.pose)
            # test
            # print("lateral_kinematics: ", lateral_kinematics)
            single_object_data.lateral_kinematics_set[current_lane] = lateral_kinematics
        
        if object_id not in self.objects_history_:
            # New Object(Create a new object in object histories)
            object_data = [single_object_data]
            self.objects_history_[object_id] = object_data
        else:
            # Object that is already in the object buffer
            object_data = self.objects_history_[object_id]
            # get previous object data and update
            prev_object_data = object_data[-1]
            self.updateLateralKinematicsVector(prev_object_data, single_object_data, self.routing_graph, cutoff_freq_of_velocity_lpf_)

            object_data.append(single_object_data)
            self.objects_history_[object_id] = object_data
    
    # line 152
    def updateLateralKinematicsVector(self, prev_obj: ObjectData, current_obj: ObjectData, routing_graph: RoutingGraph, lowpass_cutoff: float):
        current_stamp_time = self.tu.to_cpp_seconds(Time.from_msg(current_obj.header.stamp).seconds_nanoseconds())
        prev_stamp_time = self.tu.to_cpp_seconds(Time.from_msg(prev_obj.header.stamp).seconds_nanoseconds())
        dt = current_stamp_time - prev_stamp_time

        if dt < 1e-6:
            return # do not update
        
        for current_lane, current_lateral_kinematics in current_obj.lateral_kinematics_set.items():
            # 1. has same lanelet
            if current_lane in prev_obj.lateral_kinematics_set:
                prev_lateral_kinematics = prev_obj.lateral_kinematics_set[current_lane]
                self.calcLateralKinematics(prev_lateral_kinematics, current_lateral_kinematics, dt, lowpass_cutoff)
                break

            # 2. successive lanelet
            for prev_lane, prev_lateral_kinematics in prev_obj.lateral_kinematics_set.items():
                # The usage of routingRelation is different from the cpp file
                successive_lanelet = (RoutingGraph.routingRelation(routing_graph, prev_lane, current_lane) == RelationType.Successor)
                
                if successive_lanelet: # lanelet can be connected
                    self.calcLateralKinematics(prev_lateral_kinematics, current_lateral_kinematics, dt, lowpass_cutoff) # calc velocity
                    break


    def calcLateralKinematics(self, prev_lateral_kinematics: LateralKinematicsToLanelet, current_lateral_kinematics: LateralKinematicsToLanelet, dt: float, cutoff: float):
        # calc velocity via backward difference
        current_lateral_kinematics.left_lateral_velocity = (current_lateral_kinematics.dist_from_left_boundary - prev_lateral_kinematics.dist_from_left_boundary) / dt
        current_lateral_kinematics.right_lateral_velocity = (current_lateral_kinematics.dist_from_right_boundary - prev_lateral_kinematics.dist_from_right_boundary) / dt

        # low pass filtering left velocity: default cut_off is 0.6 Hz
        current_lateral_kinematics.filtered_left_lateral_velocity = self.FirstOrderLowpassFilter(
            prev_lateral_kinematics.filtered_left_lateral_velocity,
            prev_lateral_kinematics.left_lateral_velocity, current_lateral_kinematics.left_lateral_velocity,
            dt, cutoff)
        current_lateral_kinematics.filtered_right_lateral_velocity = self.FirstOrderLowpassFilter(
            prev_lateral_kinematics.filtered_right_lateral_velocity,
            prev_lateral_kinematics.right_lateral_velocity,
            current_lateral_kinematics.right_lateral_velocity, dt, cutoff)


    def FirstOrderLowpassFilter(self, prev_y: float, prev_x: float, x: float, sampling_time: float = 0.1, cutoff_freq: float = 0.1) -> float:
        # Eq:  yn = a yn-1 + b (xn-1 + xn)
        wt = 2.0 * math.pi * cutoff_freq * sampling_time
        a = (2.0 - wt) / (2.0 + wt)
        b = wt / (2.0 + wt)

        return a * prev_y + b * (prev_x + x)


    # TODO: test
    def getLanelets(self, data: LaneletsData) -> ConstLanelets:
        lanelets: ConstLanelets = []
        for lanelet_data in data:
            lanelets.append(lanelet_data.lanelet)
        
        return lanelets
    
    # TODO: test
    def initLateralKinematics(self, lanelet: ConstLanelet, pose: gmsgs.Pose) -> LateralKinematicsToLanelet:
        lateral_kinematics = LateralKinematicsToLanelet()
        left_bound = lanelet.leftBound
        right_bound = lanelet.rightBound
        left_dist = self.calcAbsLateralOffset(left_bound, pose)
        right_dist = self.calcAbsLateralOffset(right_bound, pose)

        # calc boundary distance
        lateral_kinematics.dist_from_left_boundary = left_dist
        lateral_kinematics.dist_from_right_boundary = right_dist

        # velocities are not init in the first step
        lateral_kinematics.left_lateral_velocity = 0
        lateral_kinematics.right_lateral_velocity = 0
        lateral_kinematics.filtered_left_lateral_velocity = 0
        lateral_kinematics.filtered_right_lateral_velocity = 0

        return lateral_kinematics;
    

    # TODO: test
    def calcAbsLateralOffset(self, boundary_line: ConstLineString2d, search_pose: gmsgs.Pose) -> float:
        boundary_path = []
        for point in boundary_line:
            x = point.x
            y = point.y
            boundary_path.append(self.tu.createPoint(x, y, 0.0))
        
        return abs(self.tu.calcLateralOffset(boundary_path, search_pose.position))


    # TODO: finish this method for case 2 and 3. And input lanelet_map_ptr should be lanelet::LaneletMapPtr
    # still have two inputs: object: TrackedObject, lanlet_map_ptr: LaneletMapPtr
    def changeLabelForPrediction(self, label: ObjectClassification.label, object: TrackedObject, lanelet_map: LaneletMap) -> ObjectClassification.label:
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
        self.routing_graph = RoutingGraph(self.lanelet_map, self.traffic_rules)
        self.get_logger().info('[Parellel Map Based Prediction]: Routing graph is created!')
    

    # TODO: finish this method
    def query_subtypeLanelets(self, lls: ConstLanelets, subtype) -> ConstLanelets:
        subtype_lanelets: ConstLanelets() = []
        for ll in lls:
            # print(ll.attributes)
            if ll.attributes["subtype"] == subtype:
                subtype_lanelets.append(ll)
        
        return subtype_lanelets


    def query_laneletLayer(self, ll_map: LaneletMap) -> ConstLanelets:
        lanelets: ConstLanelets() = []
        if ll_map is None:
            print("No map received!")
            return lanelets
        
        for ll in ll_map.laneletLayer:
            # Try to register the lanelet id here
            registerId(ll.id)
            lanelets.append(ll)
        
        return lanelets
    

    def query_crosswalkLanelets(self, lls: ConstLanelets) -> ConstLanelets:
        return self.query_subtypeLanelets(lls, "crosswalk")


    def query_walkwayLanelets(self, lls: ConstLanelets) -> ConstLanelets:
        return self.query_subtypeLanelets(lls, "walkway")
    

    # TODO: test
    def getCurrentLanelets(self, object: TrackedObject) -> LaneletsData:
        # obstacle point
        search_point = BasicPoint2d(object.kinematics.pose_with_covariance.pose.position.x, object.kinematics.pose_with_covariance.pose.position.y)

        # nearest lanelet
        surrounding_lanelets = l2_geom.findNearest(self.lanelet_map.laneletLayer, search_point, 10)

        # No Closest Lanelets
        if len(surrounding_lanelets) == 0:
            return []
        
        closest_lanelets: LaneletsData = []
        for lanelet in surrounding_lanelets:
            # Check if the close lanelets meet the necessary condition for start lanelets and
            # Check if similar lanelet is inside the closest lanelet
            if not self.checkCloseLaneletCondition(lanelet, object, search_point) or self.isDuplicated(lanelet, closest_lanelets):
                continue
        
            closest_lanelet = LaneletData()
            closest_lanelet.lanelet = lanelet[1]
            closest_lanelet.probability = self.calculateLocalLikelihood(lanelet[1], object)
            closest_lanelets.append(closest_lanelet)

        return closest_lanelets
    

    # TODO: test
    def checkCloseLaneletCondition(self, lanelet, object: TrackedObject, search_point: BasicPoint2d) -> bool:
        # Step1. If we only have one point in the centerline, we will ignore the lanelet
        if len(lanelet[1].centerline) <= 1:
            return False

        # Step2. Check if the obstacle is inside of this lanelet
        if not l2_geom.inside(lanelet[1], search_point):
            return False

        # If the object is in the objects history, we check if the target lanelet is
        # inside the current lanelets id or following lanelets
        object_id = self.tu.toHexString(object.object_id)
        if object_id in self.objects_history_:
            possible_lanelet = self.objects_history_[object_id][-1].future_possible_lanelets
            # TODO: check if this is correct
            not_in_possible_lanelet:bool = lanelet[1] in possible_lanelet
            if possible_lanelet and not_in_possible_lanelet:
                return False

        # Step3. Calculate the angle difference between the lane angle and obstacle angle
        object_yaw = self.tu.getYawFromQuaternion(object.kinematics.pose_with_covariance.pose.orientation)
        lane_yaw = self.tu.getLaneletAngle(lanelet[1], object.kinematics.pose_with_covariance.pose.position)
        delta_yaw = object_yaw - lane_yaw
        normalized_delta_yaw = self.tu.normalizeRadian(delta_yaw)
        abs_norm_delta = abs(normalized_delta_yaw)

        # Step4. Check if the closest lanelet is valid, and add all
        # of the lanelets that are below max_dist and max_delta_yaw
        object_vel = object.kinematics.twist_with_covariance.twist.linear.x
        is_yaw_reversed = (math.pi - delta_yaw_threshold_for_searching_lanelet_ < abs_norm_delta and object_vel < 0.0)
        if lanelet[0] < dist_threshold_for_searching_lanelet_ and (is_yaw_reversed or abs_norm_delta < delta_yaw_threshold_for_searching_lanelet_):
            return True

        return False


    # TODO: test
    def isDuplicated(self, target_lanelet, lanelets_data: LaneletsData) -> bool:
        CLOSE_LANELET_THRESHOLD = 0.1
        for lanelet_data in lanelets_data:
            target_lanelet_end_p = target_lanelet[1].centerline[-1]
            lanelet_end_p = lanelet_data.lanelet.centerline[-1]
            dist = np.hypot(target_lanelet_end_p.x - lanelet_end_p.x, target_lanelet_end_p.y - lanelet_end_p.y)

            if dist < CLOSE_LANELET_THRESHOLD:
                return True
        
        return False
    
    # TODO: test
    def calculateLocalLikelihood(self, current_lanelet: Lanelet, object: TrackedObject) -> float:
        obj_point = object.kinematics.pose_with_covariance.pose.position

        # compute yaw difference between the object and lane
        obj_yaw = self.tu.getYawFromQuaternion(object.kinematics.pose_with_covariance.pose.orientation)
        lane_yaw = self.tu.getLaneletAngle(current_lanelet, obj_point)
        delta_yaw = obj_yaw - lane_yaw
        abs_norm_delta_yaw = abs(self.tu.normalizeRadian(delta_yaw))

        # compute lateral distance
        centerline = current_lanelet.centerline
        converted_centerline = []
        for p in centerline:
            converted_p = self.tu.toGeomMsgPt(p)
            converted_centerline.append(converted_p)
        
        lat_dist = abs(self.tu.calcLateralOffset(converted_centerline, obj_point))

        # Compute Chi-squared distributed (Equation (8) in the paper)
        sigma_d = sigma_lateral_offset_
        sigma_yaw = math.pi * sigma_yaw_angle_deg_ / 180.0
        delta = np.array([lat_dist, abs_norm_delta_yaw])
        P_inv = np.array([[1.0 / (sigma_d * sigma_d), 0.0], [0.0, 1.0 / (sigma_yaw * sigma_yaw)]])
        MINIMUM_DISTANCE = 1e-6
        dist = np.maximum(np.dot(delta, np.dot(P_inv, delta)), MINIMUM_DISTANCE)

        return np.float32(1.0 / dist)
        


def main(args=None):
    rclpy.init(args=args)

    ppgn = ParellelPathGeneratorNode(time_horizon, sampling_time_interval, min_crosswalk_user_velocity)

    rclpy.spin(ppgn)



if __name__ == '__main__':
    main()