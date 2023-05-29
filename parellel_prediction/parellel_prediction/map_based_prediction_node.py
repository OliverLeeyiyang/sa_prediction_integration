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
from tf_transformations import euler_from_quaternion
import lanelet2
from lanelet2.core import LaneletMap, geometry
from lanelet2.routing import RoutingGraph

# Local imports
from .parellel_path_generator import PathGenerator
from .from_tier4_utils import Tier4Utils


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

        self.object_sub = self.create_subscription(TrackedObjects, input_topic_objects, self.object_callback, 10)
        self.map_sub = self.create_subscription(map_msgs.HADMapBin, input_topic_map, self.map_callback, 10)
        self.pred_objects_pub = self.create_publisher(PredictedObjects, pareller_output_topic, 10)
        self.lanelet_map_ptr_ = LaneletMap()
    

    def map_callback(self, msg: map_msgs.HADMapBin):
        # self.get_logger().info('[Map Based Prediction]: Start loading lanelet')
        # TODO: Load map
        # self.get_logger().info('[Map Based Prediction]: Map is loaded')
        pass


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



def main(args=None):
    rclpy.init(args=args)

    ppgn = ParellelPathGeneratorNode(time_horizon, sampling_time_interval, min_crosswalk_user_velocity)

    rclpy.spin(ppgn)



if __name__ == '__main__':
    main()