from lanelet2.io import Origin, loadRobust
from lanelet2.core import Lanelet, ConstLanelet, ConstLineString3d, BasicPoint3d, LineString3d, getId, Point3d
from lanelet2.core import LaneletMap, GPSPoint
from lanelet2.projection import UtmProjector
# from ...backup.mgrs_projector import MGRSProjector
import geopandas as gpd
import math
import numpy as np


class MapLoader:
    def __init__(self, map_file_path):
        self.lanelet2_filename = map_file_path
        #self.lanelet2_map_projector_type = "MGRS"
        self.lanelet2_map_projector_type = "UTM"
        self.center_line_resolution = 5.0
        self.lanelet2_map = LaneletMap()
    

    def load_map_for_prediction(self):
        self.lanelet2_map = self.load_map(self.lanelet2_filename, self.lanelet2_map_projector_type)
        self.overwriteLaneletsCenterline(self.lanelet2_map, self.center_line_resolution, False)
        return self.lanelet2_map
    

    def load_map(self, lanelet2_filename: str, lanelet2_map_projector_type: str) -> LaneletMap:
        if lanelet2_map_projector_type == "MGRS":
            # projector = MGRSProjector()
            # map, load_errors = loadRobust(lanelet2_filename, projector)
            # if load_errors is None:
            #    return map
            print("MGRS is not supported")
        elif lanelet2_map_projector_type == "UTM":
            map_origin_lat = 35.23808753540768
            map_origin_lon = 139.9009591876285
            position = GPSPoint(map_origin_lat, map_origin_lon)
            origin = Origin(position)
            projector = UtmProjector(origin)
            map, load_errors = loadRobust(lanelet2_filename, projector)
            # print(type(load_errors))
            if len(load_errors) == 0:
                # print("No load errors")
                return map
        else:
            print("lanelet2_map_projector_type is not supported")
            return None
        
        for error in load_errors:
            print(error)
        return None


    def overwriteLaneletsCenterline(self, lanelet_map: LaneletMap, resolution: float, force_overwrite: bool):
        for lanelet_obj in lanelet_map.laneletLayer:
            if force_overwrite or lanelet_obj.centerline is None:
                fine_center_line = self.generateFineCenterline(lanelet_obj, resolution)
                lanelet_obj.setCenterline(fine_center_line)


    def generateFineCenterline(self, lanelet_obj: ConstLanelet, resolution: float) -> LineString3d:
        # Get length of longer border
        s = gpd.GeoSeries([lanelet_obj.leftBound, lanelet_obj.rightBound])
        # left_length = s.length[0]
        # right_length = s.length[1]
        longer_distance = s.length.max()
        num_segments = max(math.ceil(longer_distance / resolution), 1)
        # Resample points
        left_points = self.resamplePoints(lanelet_obj.leftBound, num_segments)
        right_points = self.resamplePoints(lanelet_obj.rightBound, num_segments)

        # Create centerline
        centerline = LineString3d(getId())
        for i in range(num_segments+1):
            # Add ID for the average point of left and right
            center_basic_point = (right_points[i] + left_points[i]) / 2
            center_point = Point3d(getId(), center_basic_point.x(), center_basic_point.y(), center_basic_point.z())
            centerline.push_back(center_point)
        
        return centerline
    

    def resamplePoints(self, line_string: ConstLineString3d, num_segments: int):
        # Calculate length
        line_length = gpd.GeoSeries([line_string]).length[0]
        # Calculate accumulated lengths
        accumulated_lengths = self.calculateAccumulatedLengths(line_string)
        if len(accumulated_lengths) < 2:
            return []
        # Create each segment
        resampled_points: BasicPoint3d = []
        i = 0
        while i <=num_segments:
            # Find two nearest points
            target_length = (float(i) / num_segments) * line_length
            index_pair = self.find_nearest_index_pair(accumulated_lengths, target_length)

            # Apply linear interpolation
            back_point = line_string[index_pair[0]]
            front_point = line_string[index_pair[1]]
            direction_vector = front_point - back_point

            back_length = accumulated_lengths[index_pair[0]]
            front_length = accumulated_lengths[index_pair[1]]
            segment_length = front_length - back_length
            target_point = back_point + (direction_vector * (target_length - back_length) / segment_length)

            # Add to list
            resampled_points.append(target_point)
            i += 1
        
        return resampled_points
    

    def calculateAccumulatedLengths(self, line_string: ConstLineString3d):
        segment_distances = self.calculateSegmentDistances(line_string)
        accumulated_lengths = [0]
        # accumulated_lengths.extend
        accumulated_lengths += np.cumsum(segment_distances).tolist()

        return accumulated_lengths


    def calculateSegmentDistances(self, line_string: ConstLineString3d):
        segment_distances = []
        i = 1
        while i < len(line_string):
            s = gpd.GeoSeries([line_string[i]])
            s2 = gpd.GeoSeries([line_string[i - 1]])
            distance = s.distance(s2, align=False)[0]
            segment_distances.append(distance)
            i += 1
        
        return segment_distances
    

    def find_nearest_index_pair(self, accumulated_lengths: list, target_length: float):
        # List size
        N = len(accumulated_lengths)

        # Front
        if target_length < accumulated_lengths[1]:
            return (0, 1)

        # Back
        if target_length > accumulated_lengths[N - 2]:
            return (N - 2, N - 1)

        # Middle
        for i in range(1, N):
            if (accumulated_lengths[i - 1] <= target_length <= accumulated_lengths[i]):
                return (i - 1, i)

        # Throw an exception because this never happens
        raise RuntimeError("No nearest point found.")




def main():
    pass



if __name__ == '__main__':
    main()