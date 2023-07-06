# System and Projects imports
import unique_identifier_msgs.msg._uuid as uuid
import geometry_msgs.msg as gmsgs
import tf2_geometry_msgs as tf2_gmsgs

# Outside imports
import tf_transformations 
import math
import numpy as np

from lanelet2.core import ConstLanelet, BasicPoint2d, ConstLineString3d, LineString3d, Point3d, getId, ConstPoint3d, ConstPoint2d
import lanelet2.geometry as l2_geom



class Tier4Utils():
    '''Methods for map_based_prediction_node.'''
    def __init__(self):
        # print('Tier4Utils class is ready!')
        pass
    

    # Methods:
    # test passed
    def calcoffsetpose(self, p: gmsgs.Pose, x: float, y: float, z: float) -> gmsgs.Pose:
        new_pose = gmsgs.Pose()
        transform = gmsgs.TransformStamped()
        transform.transform.translation = self.createTranslation(x, y, z)
        transform.transform.rotation = self.createQuaternion(0.0, 0.0, 0.0, 1.0)

        tf_offset = tf2_gmsgs.from_msg_msg(transform)
        tf_pose = tf2_gmsgs.from_msg_msg(p)
        new_pose = tf2_gmsgs.do_transform_pose(tf_pose, tf_offset)

        return new_pose
    
    # test passed
    def createTranslation(self, x: float, y: float, z: float) -> gmsgs.Vector3:
        v = gmsgs.Vector3()
        v.x = x
        v.y = y
        v.z = z

        return v

    # test passed
    def createQuaternion(self, x: float, y: float, z: float, w: float) -> gmsgs.Quaternion:
        q = gmsgs.Quaternion()
        q.x = x
        q.y = y
        q.z = z
        q.w = w

        return q
    
    # test passed
    # Self defined methods to get yaw from quaternion
    def getYawFromQuaternion(self, q: gmsgs.Quaternion) -> float:
        # q_array = [q.x, q.y, q.z, q.w]
        # euler = tf_transformations.euler_from_quaternion(q_array)
        # return euler[2]
        sqx = q.x * q.x
        sqy = q.y * q.y
        sqz = q.z * q.z
        sqw = q.w * q.w

        sarg = -2.0 * (q.x * q.z - q.w * q.y) / (sqx + sqy + sqz + sqw)

        if sarg <= -0.99999:
            yaw = -2.0 * math.atan2(q.y, q.x)
        elif sarg >= 0.99999:
            yaw = 2.0 * math.atan2(q.y, q.x)
        else:
            yaw = math.atan2(2 * (q.x * q.y + q.w * q.z), sqw + sqx - sqy - sqz)
        
        return yaw
    

    def toHexString(self, id:uuid.UUID) -> str:
        hex_string = ""
        for i in range(16):
            hex_string += format(id.uuid[i], '02x')
        return hex_string
    
    # test passed
    def createQuaternionFromYaw(self, yaw: float) -> gmsgs.Quaternion:
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        qq = self.createQuaternion(q[0], q[1], q[2], q[3])
        
        return qq
    
    # test passed
    def calcAzimuthAngle(self, p_from: gmsgs.Point, p_to: gmsgs.Point) -> float:
        dx = p_to.x - p_from.x
        dy = p_to.y - p_from.y

        return math.atan2(dy, dx)
    
    # test passed
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
        
        p_front = self.getPoint(overlap_removed_points[seg_idx])
        p_back = self.getPoint(overlap_removed_points[seg_idx + 1])

        segment_vec = np.array([p_back.x - p_front.x, p_back.y - p_front.y, 0.0])
        target_vec = np.array([p_target.x - p_front.x, p_target.y - p_front.y, 0.0])

        return np.dot(segment_vec, target_vec) / np.linalg.norm(segment_vec)

    # test passed
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

    # test passed
    def findNearestSegmentIndex(self, points, point: gmsgs.Point) -> int:
        '''find nearest segment index to point.

        Segment is straight path between two continuous points of trajectory.

        When point is on a trajectory point whose index is nearest_idx, return nearest_idx - 1.

        When input is a gmsgs.pose, write a new method(trajactory.hpp line 380)
        '''
        nearest_idx = self.findNearestIndex(points, point)

        if nearest_idx == 0:
            return 0
        if nearest_idx == len(points) - 1:
            return len(points) - 2
        
        signed_length = self.calcLongitudinalOffsetToSegment(points, nearest_idx, point)

        if signed_length <= 0:
            return nearest_idx - 1
        
        return nearest_idx

    # test passed
    def findNearestIndex(self, points, point: gmsgs.Point) -> int:
        '''When input is a gmsgs.pose, write a new method(trajactory.hpp line 380)'''
        self.validateNonEmpty(points)

        min_dist = float('inf')
        min_idx = 0

        for i in range(len(points)):
            dist = self.calcSquaredDistance2d(points[i], point)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
            
        return min_idx
    
    # test passed
    def validateNonEmpty(self, points):
        if len(points) == 0:
            raise ValueError("Points is empty")
    
    # test passed
    def getPoint(self, point: gmsgs.Point | gmsgs.Pose) -> gmsgs.Point:
        match type(point):
            case gmsgs.Point:
                return point
            case gmsgs.Pose:
                return point.position
            case _:
                raise TypeError("point must be Point or Pose")
        
    # test passed
    def calcSquaredDistance2d(self, point1, point2) -> float:
        p1 = self.getPoint(point1)
        p2 = self.getPoint(point2)
        return (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2
    
    # test passed
    def calcDistance2d(self, point1, point2) -> float:
        p1 = self.getPoint(point1)
        p2 = self.getPoint(point2)
        return np.hypot(p1.x - p2.x, p1.y - p2.y)
    
    # test passed
    def calcDistance3d(self, point1, point2) -> float:
        p1 = self.getPoint(point1)
        p2 = self.getPoint(point2)
        return np.hypot(self.calcDistance2d(point1, point2), p1.z - p2.z)
    
    # test passed
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
    
    # test passed
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
        # print("seg_idx: ", seg_idx)

        return self.calcLateralOffset_later(points, p_target, seg_idx, throw_exception)
    
    # test passed
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
        
        p_front = self.getPoint(overlap_removed_points[seg_idx])
        p_back = self.getPoint(overlap_removed_points[seg_idx + 1])

        segment_vec = np.array([p_back.x - p_front.x, p_back.y - p_front.y, 0.0])
        target_vec = np.array([p_target.x - p_front.x, p_target.y - p_front.y, 0.0])

        cross_vec = np.cross(segment_vec, target_vec)
        return cross_vec[2] / np.linalg.norm(segment_vec)
    
    # TODO: test
    def getLaneletAngle(self, lanelet: ConstLanelet, search_point: gmsgs.Point) -> float:
        llt_search_point = BasicPoint2d(search_point.x, search_point.y)
        segment = self.getClosestSegment(llt_search_point, lanelet.centerline)

        return math.atan2(segment[1].y - segment[0].y, segment[1].x - segment[0].x)
    
    # TODO: test
    def getClosestSegment(self, search_pt: BasicPoint2d, linestring: ConstLineString3d) -> ConstLineString3d:
        if len(linestring) < 2:
            raise LineString3d()
        
        # closest_segment: ConstLineString3d()
        min_distance = float("inf")

        for i in range(1, len(linestring)):
            prev_basic_pt = linestring[i - 1].basicPoint()
            current_basic_pt = linestring[i].basicPoint()

            prev_pt = Point3d(
                0, prev_basic_pt.x, prev_basic_pt.y, prev_basic_pt.z
            )
            current_pt = Point3d(
                0, current_basic_pt.x, current_basic_pt.y, current_basic_pt.z
            )

            current_segment = LineString3d(0, [prev_pt, current_pt])
            distance = l2_geom.distance(l2_geom.to2D(current_segment), search_pt) # Maybe don't need to use basicLineString
            if distance < min_distance:
                closest_segment = current_segment
                min_distance = distance

        return closest_segment
    
    # TODO: test
    def normalizeRadian(self, rad: float, min_rad: float = -math.pi) -> float:
        max_rad = min_rad + 2 * math.pi
        value = rad % (2 * math.pi)

        if min_rad <= value < max_rad:
            return value

        return value - math.copysign(2 * math.pi, value)
    

    # TODO: test
    def toGeomMsgPt(self, src) -> gmsgs.Point:
        if type(src) == ConstPoint3d:
            return gmsgs.Point(x=src.x, y=src.y, z=src.z)
        elif type(src) == ConstPoint2d:
            return gmsgs.Point(x=src.x, y=src.y, z=0.0)
        elif type(src) == gmsgs.Point32:
            return gmsgs.Point(x=src.x, y=src.y, z=src.z)
        else:
            return gmsgs.Point(x=src.x, y=src.y, z=src.z)
    

    def createPoint(self, x: float, y: float, z: float) -> gmsgs.Point:
        p = gmsgs.Point()
        p.x = x
        p.y = y
        p.z = z

        return p



def main():
    pass



if __name__ == '__main__':
    main()