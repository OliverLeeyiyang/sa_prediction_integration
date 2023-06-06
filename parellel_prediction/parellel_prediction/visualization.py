import rclpy
from rclpy.node import Node
from autoware_auto_perception_msgs.msg import TrackedObjects
from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_auto_perception_msgs.msg import PredictedPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Followed topics
topic_objects = '/perception/object_recognition/objects'
pareller_output_topic = '/parellel/objects'

# Topics to publish
original_path_pub = '/visualization/original_path'
parellel_path_pub = '/visualization/parellel_path'

# Debug topics
debug_topic = '/perception/object_recognition/tracking/objects'


class Visualization(Node):
    def __init__(self):
        super().__init__('path_visualization_node')
        self.original_path_pub = self.create_publisher(Path, original_path_pub, 10)
        self.parellel_path_pub = self.create_publisher(Path, parellel_path_pub, 10)
        self.subscription1 = self.create_subscription(
            PredictedObjects,
            topic_objects,
            self.callback1,
            10)
        self.subscription2 = self.create_subscription(
            PredictedObjects,
            pareller_output_topic,
            self.callback2,
            10)
        self.subscription3 = self.create_subscription(
            TrackedObjects,
            debug_topic,
            self.callback3,
            10)
            

    def callback3(self, msg: TrackedObjects):
        for obj in msg.objects:
            obj_pose = obj.kinematics.pose_with_covariance.pose
            print('cpp obj pose:         ', obj_pose)
    
    def callback1(self, msg: PredictedObjects):
        path1 = Path()
        path1.header = msg.header
        for obj in msg.objects:
            pre_pose = obj.kinematics.predicted_paths[0].path[0]
            print('cpp pred obj pose:    ', pre_pose)
            for pre_path in obj.kinematics.predicted_paths:
                # if pose is not empty
                if pre_path.path is None:
                    print('path is empty')
                    continue
                else:
                    for pose in pre_path.path:
                        poses = PoseStamped()
                        poses.pose = pose
                        path1.poses.append(poses)

        self.original_path_pub.publish(path1)
    

    def callback2(self, msg: PredictedObjects):
        path2 = Path()
        path2.header = msg.header
        for obj in msg.objects:
            pre_pose = obj.kinematics.predicted_paths[0].path[0]
            print('python pred obj pose: ', pre_pose)
            for pre_path in obj.kinematics.predicted_paths:
                # if pose is not empty
                if pre_path.path is None:
                    print('path is empty')
                    continue
                else:
                    for pose in pre_path.path:
                        poses = PoseStamped()
                        poses.pose = pose
                        path2.poses.append(poses)

        self.parellel_path_pub.publish(path2)


def main(args=None):
    rclpy.init(args=args)
    visualization = Visualization()
    rclpy.spin(visualization)

if __name__ == '__main__':
    main()