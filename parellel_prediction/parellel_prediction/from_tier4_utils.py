import unique_identifier_msgs.msg._uuid as uuid
import geometry_msgs.msg as gmsgs
import tf2_geometry_msgs as tf2_gmsgs
import tf_transformations 
import math


class Tier4Utils():
    '''Methods for map_based_prediction_node.'''
    def __init__(self):
        print('Tier4Utils class is ready!')
    

    # Methods:
    def toHexString(self, id:uuid.UUID) -> str:
        hex_string = ""
        for i in range(16):
            hex_string += format(id.uuid[i], '02x')
        return hex_string
    

    def createQuaternionFromYaw(yaw: float) -> gmsgs.Quaternion:
        q = gmsgs.Quaternion()
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

        return tf2_gmsgs.to_msg_msg(q)
    

    def calcAzimuthAngle(p_from: gmsgs.Point, p_to: gmsgs.Point) -> float:
        dx = p_to.x - p_from.x
        dy = p_to.y - p_from.y
        return math.atan2(dy, dx)



def main():
    pass


if __name__ == '__main__':
    main()