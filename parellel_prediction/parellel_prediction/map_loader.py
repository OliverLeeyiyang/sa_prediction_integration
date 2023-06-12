from lanelet2.io import Origin, loadRobust
from lanelet2.core import Lanelet
from lanelet2.core import LaneletMap, GPSPoint
from lanelet2.projection import UTMProjector
import rclpy
from .mgrs_projector import MGRSProjector


class MapLoader:
    def __init__(self, map_file_path):
        self.lanelet2_filename = map_file_path
        self.lanelet2_map_projector_type = "MGRS"
        self.center_line_resolution = 5.0
        self.lanelet2_map = None
    

    def load_map_for_prediction(self):
        self.lanelet2_map = self.load_map(self.lanelet2_filename, self.lanelet2_map_projector_type)
        self.overwriteLaneletsCenterline(self.lanelet2_map, self.center_line_resolution, False)
        return self.lanelet2_map
    

    def load_map(self, lanelet2_filename: str, lanelet2_map_projector_type: str) -> LaneletMap:
        if lanelet2_map_projector_type == "MGRS":
            projector = MGRSProjector()
            map, load_errors = loadRobust(lanelet2_filename, projector)
            if load_errors is None:
                return map
        elif lanelet2_map_projector_type == "UTM":
            map_origin_lat = 0.0
            map_origin_lon = 0.0
            position = GPSPoint(map_origin_lat, map_origin_lon, 0.0)
            origin = Origin(position)
            projector = UTMProjector(origin)
            map, load_errors = loadRobust(lanelet2_filename, projector)
            if load_errors is None:
                return map
        else:
            logger = rclpy.logging.get_logger("map_loader")
            rclpy.logging.error(logger, "lanelet2_map_projector_type is not supported")
            return None
        
        for error in load_errors:
            rclpy.logging.error_stream(logger, error)
        return None


    def overwriteLaneletsCenterline(self, lanelet_map: LaneletMap, resolution: float, force_overwrite: bool):
        for lanelet_obj in lanelet_map.laneletLayer:
            if force_overwrite or not lanelet_obj.hasCustomCenterline():
                fine_center_line = self.generateFineCenterline(lanelet_obj, resolution)
                lanelet_obj.setCenterline(fine_center_line)


    def generateFineCenterline(self, lanelet_obj: Lanelet, resolution: float):
        pass



def main():
    pass



if __name__ == '__main__':
    main()