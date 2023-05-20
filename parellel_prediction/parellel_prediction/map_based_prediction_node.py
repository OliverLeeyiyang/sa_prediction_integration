from .parellel_path_generator import PathGenerator

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
sampling_time_interval = 0.1
min_crosswalk_user_velocity = 0.1



class ParellelPathGeneratorNode(Node):
    ''' Node for generating path for other vehicles and crosswalk users.
        Input topics:   /perception/object_recognition/tracking/objects
                        /map/vector_map
        Output topics:  /perception/object_recognition/objects'''

    def __init__(self, time_horizon_, sampling_time_interval_, min_crosswalk_user_velocity_):
        super().__init__('parellel_path_generator_node')

        self.ppgn = PathGenerator(time_horizon_, sampling_time_interval_, min_crosswalk_user_velocity_)

        self.object_sub = self.create_subscription(TrackedObjects, input_topic_objects, self.object_callback, 10)
        self.pred_objects_pub = self.create_publisher(PredictedObjects, output_topic_objects, 10)
    

    def object_callback(self, msg: TrackedObjects):
        self.object = msg.objects[0]

        #test
        self.test_path = self.generateStraightPath(self.object)
        self.test_path_pub.publish(self.test_path)


def main():
    pass


if __name__ == '__main__':
    main()