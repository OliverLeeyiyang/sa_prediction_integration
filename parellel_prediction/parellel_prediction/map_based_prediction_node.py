from .parellel_path_generator import PathGenerator

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

    Input topics  :  /perception/object_recognition/tracking/objects and /map/vector_map

    Output topics : /perception/object_recognition/objects

    --------------------
    '''

    def __init__(self, time_horizon_, sampling_time_interval_, min_crosswalk_user_velocity_):
        super().__init__('parellel_path_generator_node')

        self.pg = PathGenerator(time_horizon_, sampling_time_interval_, min_crosswalk_user_velocity_)

        self.object = TrackedObject()

        self.object_sub = self.create_subscription(TrackedObjects, input_topic_objects, self.object_callback, 10)
        self.pred_objects_pub = self.create_publisher(PredictedObjects, output_topic_objects, 10)

        # Test topic for predicted path: PredictedPath()
        self.path_pub = self.create_publisher(PredictedPath, '/test_pred_path', 10)
        self.flag = False
    

    def object_callback(self, msg: TrackedObjects):
        self.object = msg.objects

        # Test generateStraightPath
        pred_path = self.pg.generateStraightPath(self.object)
        self.flag = True
        print('got pred path!')
        self.path_pub.publish(pred_path)



def main(args=None):
    rclpy.init(args=args)

    ppgn = ParellelPathGeneratorNode(time_horizon, sampling_time_interval, min_crosswalk_user_velocity)

    rclpy.spin(ppgn)



if __name__ == '__main__':
    main()