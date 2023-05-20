from .parellel_path_generator import *
from .self_utils import SelfUtils
import rclpy



class TestClass():
    '''Test methods in SelfUtils class and ParellelPathGeneratorNode class'''

    def __init__(self):
        self.su = SelfUtils()
        # self.ppgn = ParellelPathGeneratorNode(time_horizon, sampling_time_interval, min_crosswalk_user_velocity)


    def test_method_in_selfutils(self):
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
        new_pose = self.su.calc_offset_pose(self.pose, self.x, self.y, self.z)
        print('New pose is: ', new_pose)
    

    def test_ppgn(self):
        pass


def main():
    rclpy.init()
    tc = TestClass()
    print('Hi from pp_test.py')
    tc.test_method_in_selfutils()

    # rclpy.spin(tc)
    # tc.destroy_node()
    # rclpy.shutdown()



if __name__ == '__main__':
    main()