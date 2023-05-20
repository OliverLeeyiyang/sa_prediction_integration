import tf2_geometry_msgs as tf2_gmsgs
import geometry_msgs.msg as gmsgs


class SelfUtils():
    '''Methods for PathGenerator class, tested in pp_test.py.'''
    def __init__(self):
        print('SelfUtils class is ready!')


    # Methods:
    def calc_offset_pose(self, p: gmsgs.Pose, x: float, y: float, z: float) -> gmsgs.Pose:
        new_pose = gmsgs.Pose()
        transform = gmsgs.TransformStamped()
        transform.transform.translation = self.createTranslation(x, y, z)
        transform.transform.rotation = self.createQuaternion(0.0, 0.0, 0.0, 1.0)

        tf_offset = tf2_gmsgs.from_msg_msg(transform)
        tf_pose = tf2_gmsgs.from_msg_msg(p)
        new_pose = tf2_gmsgs.do_transform_pose(tf_pose, tf_offset)

        return new_pose


    def createTranslation(self, x: float, y: float, z: float) -> gmsgs.Vector3:
        v = gmsgs.Vector3()
        v.x = x
        v.y = y
        v.z = z

        return v


    def createQuaternion(self, x: float, y: float, z: float, w: float) -> gmsgs.Quaternion:
        q = gmsgs.Quaternion()
        q.x = x
        q.y = y
        q.z = z
        q.w = w

        return q



def main():
    pass


if __name__ == '__main__':
    main()