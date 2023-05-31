# System and Projects imports
import rclpy
from rclpy.node import Node

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

# Outside imports
import math
import pickle
import lanelet2
import io
from typing import List
from lanelet2.core import LaneletMap, ConstLanelet, Lanelet, registerId
from lanelet2.routing import RoutingGraph
from lanelet2.traffic_rules import TrafficRules
import lanelet2.traffic_rules as traffic_rules

# Local imports
from .parellel_path_generator import PathGenerator
from .from_tier4_utils import Tier4Utils
ConstLanelets = List[ConstLanelet]
Lanelets = List[Lanelet]

# input topics
input_topic_objects = '/perception/object_recognition/tracking/objects'
input_topic_map = '/map/vector_map'

# output topics
# output_topic_objects = '/perception/object_recognition/objects'
pareller_output_topic = '/parellel/objects'

# Parameters
time_horizon = 10.0
sampling_time_interval = 0.5
min_crosswalk_user_velocity = 1.0
# Maybe use Node.declare_parameter() to get parameters from launch file



class ParellelPathGeneratorNode(Node):
    '''Node for generating path for other vehicles and crosswalk users.

    Launch simulation:

    cd ~/ma_prediction_integration/src/parellel_prediction/launch

    ros2 launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

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
        self.traffic_rules = None
        self.routing_graph = None
    

    def map_callback(self, msg: map_msgs.HADMapBin):
        self.get_logger().info('[Parellel Map Based Prediction]: Start loading lanelet')
        self.fromBinMsg(msg)
        self.get_logger().info('[Parellel Map Based Prediction]: Map is loaded')
        
        all_lanelets = self.query_laneletLayer(self.lanelet_map)
        # TODO: Get all crosswalks and walkways, line 552-555


    def object_callback(self, in_objects: TrackedObjects):
        # TODO: Guard for map pointer and frame transformation(line 561)

        # TODO: world,map and bask_link transform
        # TODO: Remove old objects information in object history(line 582)

        # result output
        output = PredictedObjects()
        output.header = in_objects.header
        output.header.frame_id = 'map'

        # Didn't do: debug_markers

        # Deal with each object
        for object in in_objects.objects:
            # seems like this is not necessary
            object_id = self.tu.toHexString(object.object_id)

            transformed_object = TrackedObject()
            transformed_object = object

            # TODO: transform object frame if it's not based on map frame(line 599), need to use transform

            # get tracking label and update it for the prediction
            #tracking_label = transformed_object.classification.label
            tracking_label = 1
            label = self.changeLabelForPrediction(tracking_label)

            # TODO: For crosswalk user, don't consider this situation now.(line 612)
            if label == ObjectClassification.PEDESTRIAN or label == ObjectClassification.BICYCLE:
                continue
            # For road user
            elif label == ObjectClassification.CAR or label == ObjectClassification.BUS or label == ObjectClassification.TRAILER\
                    or label == ObjectClassification.MOTORCYCLE or label == ObjectClassification.TRUCK:
                # Update object yaw and velocity
                # self.updateObjectData(transformed_object)

                # TODO: Get Closest Lanelet (line 624)
                # current_lanelets = 

                # TODO: Update Objects History (line 627)

                # TODO: For off lane obstacles

                # TODO: For too-slow vehicle

                # Get Predicted Reference Path for Each Maneuver and current lanelets
                # return: <probability, paths>

                # If predicted reference path is empty, assume this object is out of the lane

                # Didn't do: Get Debug Marker for On Lane Vehicles

                # TODO: Generate Predicted Path

                # TODO: Normalize Path Confidence and output the predicted object

                # Test
                predicted_object = self.convertToPredictedObject(transformed_object)
                predicted_path = self.pg.generatePathForNonVehicleObject(transformed_object)
                predicted_path.confidence = 1.0

                predicted_object.kinematics.predicted_paths.append(predicted_path)
                output.objects.append(predicted_object)

            # For unknown object (line 725)
            # Use this part to test PathGenerator.generateStraightPath
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


    def updateObjectData(self, object: TrackedObject):
        if object.kinematics.orientation_availability == DetectedObjectKinematics.AVAILABLE:
            return
        
        # Compute yaw angle from the velocity and position of the object
        object_pose = object.kinematics.pose_with_covariance.pose
        object_twist = object.kinematics.twist_with_covariance.twist
        future_object_pose = self.tu.calcoffsetpose(object_pose, object_twist.linear.x * 0.1, object_twist.linear.y * 0.1, 0.0)

        if object_twist.linear.x < 0.0:
            if object.kinematics.orientation_availability == DetectedObjectKinematics.SIGN_UNKNOWN:
                # original_yaw = euler_from_quaternion(object.kinematics.pose_with_covariance.pose.orientation)[2]
                original_yaw = self.tu.getYawFromQuaternion(object.kinematics.pose_with_covariance.pose.orientation)
                # flip the angle
                object.kinematics.pose_with_covariance.pose.orientation = self.tu.createQuaternionFromYaw(original_yaw + math.pi)
            else:
                update_object_yaw = self.tu.calcAzimuthAngle(object_pose.position, future_object_pose.position)
                object.kinematics.pose_with_covariance.pose.orientation = self.tu.createQuaternionFromYaw(update_object_yaw)
            
            object.kinematics.twist_with_covariance.twist.linear.x *= -1.0
        
        return
    

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
        if map is None:
            print("fromBinMsg: map is null pointer!")
            return
        
        # TODO: test!
        # data_str = msg.data.tobytes()
        # self.lanelet_map = pickle.loads(data_str)
        # id_counter: lanelet2.Id = 0
        ss = io.BytesIO(msg.data)
        self.lanelet_map = pickle.loads(ss.read())

        id_counter: lanelet2.Id = 0
        id_counter = self.lanelet_map
        registerId(id_counter)
        self.get_logger().info('[Parellel Map Based Prediction]: Id is registered!')
        
        self.traffic_rules = traffic_rules.create(traffic_rules.Locations.Germany, traffic_rules.Participants.Vehicle)
        self.routing_graph = RoutingGraph.build(map, self.traffic_rules)


    def query_laneletLayer(self, ll_map: LaneletMap) -> ConstLanelets:
        lanelets = ConstLanelets()
        if ll_map is None:
            print("No map received!")
            return lanelets
        
        for ll in ll_map.laneletLayer:
            lanelets.append(ll)
        
        return lanelets
    

    def query_crosswalkLanelets(self, lls: ConstLanelets) -> ConstLanelets:
        pass


def main(args=None):
    rclpy.init(args=args)

    ppgn = ParellelPathGeneratorNode(time_horizon, sampling_time_interval, min_crosswalk_user_velocity)

    rclpy.spin(ppgn)



if __name__ == '__main__':
    main()