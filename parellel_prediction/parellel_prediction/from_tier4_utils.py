# System and Projects imports
import unique_identifier_msgs.msg._uuid as uuid
import geometry_msgs.msg as gmsgs
import tf2_geometry_msgs as tf2_gmsgs

# Outside imports
import tf_transformations 
import math
import numpy as np



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
    

    def getYawFromQuaternion(q: gmsgs.Quaternion) -> float:
        euler = tf_transformations.euler_from_quaternion(q)

        return euler[2]
    

    def findNearestSegmentIndex(self, points, point: gmsgs.Point) -> int:
        '''find nearest segment index to point.

        Segment is straight path between two continuous points of trajectory.

        When point is on a trajectory point whose index is nearest_idx, return nearest_idx - 1.
        '''
        nearest_idx = self.findNearestIndex(points, point)

        if nearest_idx == 0:
            return 0
        if nearest_idx == len(points) - 1:
            return len(points) - 2
    

    def calcLongitudinalOffsetToSegment(self, points, seg_idx: int, p_target: gmsgs.Point, throw_exception: bool = False) -> float:
        '''calculate longitudinal offset (length along trajectory from seg_idx point to nearest point to p_target on trajectory).
        
        If seg_idx point is after that nearest point, length is negative.

        Segment is straight path between two continuous points of trajectory.
        '''
        if seg_idx >= len(points) - 1:
            if throw_exception:
                raise IndexError("Segment index is invalid.")
            return np.nan
        
        overlap_removed_points = self.removeOverlapPoints(points, seg_idx)

        if throw_exception:
            self.validateNonEmpty(overlap_removed_points)
        else:
            try:
                self.validateNonEmpty(overlap_removed_points)
            except Exception as e:
                print(e)
                return np.nan
        
        if seg_idx >= len(overlap_removed_points) - 1:
            if throw_exception:
                raise RuntimeError("Same points are given.")
            return np.nan
        
        p_front = overlap_removed_points[seg_idx]
        p_back = overlap_removed_points[seg_idx + 1]

        segment_vec = np.array([p_back.x - p_front.x, p_back.y - p_front.y, 0.0])
        target_vec = np.array([p_target.x - p_front.x, p_target.y - p_front.y, 0.0])

        return np.dot(segment_vec, target_vec) / np.linalg.norm(segment_vec)


    def removeOverlapPoints(self, points, start_idx: int = 0) -> list:
        if len(points) < start_idx + 1:
            return points
        
        points_type = type(points)
        dst = points_type()

        for i in range(start_idx + 1):
            dst.append(points[i])
        
        eps = 1e-8
        for i in range(start_idx + 1, len(points)):
            prev_p = dst[-1]
            curr_p = points[i]
            dist = self.calcDistance2d(prev_p, curr_p)
            if dist < eps:
                continue
            dst.append(points[i])
        
        return dst


    def findNearestIndex(self, points, point: gmsgs.Point) -> int:
        self.validateNonEmpty(points)

        min_dist = float('inf')
        min_idx = 0

        for i in range(len(points)):
            dist = self.calcSquaredDistance2d(points[i], point)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
            
        return min_idx
    

    def validateNonEmpty(self, points):
        if len(points) == 0:
            raise ValueError("Points is empty")
        
    
    def calcSquaredDistance2d(self, point1, point2) -> float:
        return (point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2
    

    def calcDistance2d(self, point1, point2) -> float:
        return np.hypot(point1.x - point2.x, point1.y - point2.y)
    

    def calcDistance3d(self, point1, point2) -> float:
        return np.hypot(self.calcDistance2d(point1, point2), point1.z - point2.z)
    

    def calcSignedArcLength(self, points, src_idx: int, dst_idx: int) -> float:
        try:
            self.validateNonEmpty(points)
        except Exception as e:
            print(e)
            return 0.0
        
        if src_idx > dst_idx:
            return -self.calcSignedArcLength(points, dst_idx, src_idx)
        
        dist_sum = 0.0
        for i in range(src_idx, dst_idx):
            dist_sum += self.calcDistance2d(points[i], points[i + 1])
        
        return dist_sum
    

    def calcLateralOffset(self, points, p_target: gmsgs.Point, throw_exception: bool = False) -> float:
        '''calculate lateral offset from p_target (length from p_target to trajectory).

        The function gets the nearest segment index between the points of trajectory and the given target point, 
        then uses that segment index to calculate lateral offset. Segment is straight path between two continuous points of trajectory.
        '''
        overlap_removed_points = self.removeOverlapPoints(points, 0)

        if throw_exception:
            self.validateNonEmpty(overlap_removed_points)
        else:
            try:
                self.validateNonEmpty(overlap_removed_points)
            except Exception as e:
                print(e)
                return np.nan
        
        if len(overlap_removed_points) == 1:
            if throw_exception:
                raise RuntimeError("Same points are given.")
            
        seg_idx = self.findNearestSegmentIndex(overlap_removed_points, p_target)

        return self.calcLateralOffset_later(points, p_target, seg_idx, throw_exception)
    

    def calcLateralOffset_later(self, points, p_target: gmsgs.Point, seg_idx: int, throw_exception: bool = False) -> float:
        '''calculate lateral offset from p_target (length from p_target to trajectory) using given segment index. 
        
        Segment is straight path between two continuous points of trajectory.
        '''
        overlap_removed_points = self.removeOverlapPoints(points, 0)

        if throw_exception:
            self.validateNonEmpty(overlap_removed_points)
        else:
            try:
                self.validateNonEmpty(overlap_removed_points)
            except Exception as e:
                print(e)
                return np.nan
        
        if len(overlap_removed_points) == 1:
            if throw_exception:
                raise RuntimeError("Same points are given.")
        
        p_front = overlap_removed_points[seg_idx]
        p_back = overlap_removed_points[seg_idx + 1]

        segment_vec = np.array([p_back.x - p_front.x, p_back.y - p_front.y, 0.0])
        target_vec = np.array([p_target.x - p_front.x, p_target.y - p_front.y, 0.0])

        cross_vec = np.cross(segment_vec, target_vec)
        return cross_vec[2] / np.linalg.norm(segment_vec)
        





def main():
    pass



if __name__ == '__main__':
    main()