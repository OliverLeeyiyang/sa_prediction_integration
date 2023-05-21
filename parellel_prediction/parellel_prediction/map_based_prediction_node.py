from .parellel_path_generator import PathGenerator
from .from_tier4_utils import Tier4Utils

import rclpy
from rclpy.node import Node

# namespeces in cpp file
from autoware_auto_perception_msgs.msg import ObjectClassification
from autoware_auto_perception_msgs.msg import PredictedObject
from autoware_auto_perception_msgs.msg import PredictedObjectKinematics
from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_auto_perception_msgs.msg import PredictedPath
from autoware_auto_perception_msgs.msg import TrackedObject
from autoware_auto_perception_msgs.msg import TrackedObjectKinematics
from autoware_auto_perception_msgs.msg import TrackedObjects

import autoware_auto_mapping_msgs.msg as map_msgs


# input topics
input_topic_objects = '/perception/object_recognition/tracking/objects'
input_topic_map = '/map/vector_map'

# output topics
output_topic_objects = '/perception/object_recognition/objects'

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

    --------------------
    '''

    def __init__(self, time_horizon_, sampling_time_interval_, min_crosswalk_user_velocity_):
        super().__init__('parellel_path_generator_node')

        self.pg = PathGenerator(time_horizon_, sampling_time_interval_, min_crosswalk_user_velocity_)
        self.tu = Tier4Utils()

        self.object_sub = self.create_subscription(TrackedObjects, input_topic_objects, self.object_callback, 10)
        self.map_sub = self.create_subscription(map_msgs.HADMapBin, input_topic_map, self.map_callback, 10)
        self.pred_objects_pub = self.create_publisher(PredictedObjects, output_topic_objects, 10)

        # Test topic for predicted path: PredictedPath()
        self.path_pub = self.create_publisher(PredictedPath, '/test_pred_path', 10)
        self.flag = False
    

    def map_callback(self, msg: map_msgs.HADMapBin):
        self.get_logger().info('[Map Based Prediction]: Start loading lanelet')
        # TODO: Load map
        self.get_logger().info('[Map Based Prediction]: Map is loaded')


    def object_callback(self, in_objects: TrackedObjects):
        # TODO: Guard for map pointer and frame transformation(line 561)
        # TODO: world,map and bask_link transform
        # TODO: Remove old objects information in object history(line 582)

        # result output
        output = PredictedObjects()
        output.header = in_objects.header
        output.header.frame_id = 'map'

        # Didn't do: debug_markers

        # Deal woth each object
        for object in in_objects.objects:
            object_id = self.tu.toHexString(object.object_id)
            transformed_object = TrackedObject()
            # transformed_object = object

            # TODO: transform object frame if it's based on map frame(line 599)

            # get tracking label and update it for the prediction
            tracking_label = transformed_object.classification.label
            label = self.changeLabelForPrediction(tracking_label)

            # For crosswalk user, don't consider this situation now.(line 612)
            if label == ObjectClassification.PEDESTRIAN or label == ObjectClassification.BICYCLE:
                continue
            # For road user
            elif label == ObjectClassification.CAR or label == ObjectClassification.BUS or label == ObjectClassification.TRAILER\
                    or label == ObjectClassification.MOTORCYCLE or label == ObjectClassification.TRUCK:
                # TODO: A lot of things to do(line 620-722)
                continue
            # For unknown. (line 725)
            else:
                continue
        
        # Publish results
        self.pred_objects_pub.publish(output)
            

    # TODO: finish this method for case 2 and 3. And input lanelet_map_ptr should be lanelet::LaneletMapPtr
    def changeLabelForPrediction(self, label: ObjectClassification.label, object: TrackedObject, lanlet_map_ptr) -> ObjectClassification.label:
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