import sys
import math
import geographiclib
from lanelet2.projection import Projector
from lanelet2.core import GPSPoint, BasicPoint3d
from lanelet2.io import Origin



class MGRSProjector:
    def __init__(self, origin = Origin(0.0, 0.0)):
        super().__init__(Projector(origin))
        self.origin = origin
        self.projected_grid_ = ""
    
    def forward(self, gps: GPSPoint) -> BasicPoint3d:
        mgrs_point = self.forward(gps, 0)
        return mgrs_point
    
    def forward(self, gps: GPSPoint, precision: int) -> BasicPoint3d:
        prev_projected_grid = self.projected_grid_

        mgrs_point = BasicPoint3d(0.0, 0.0, gps.ele)
        utm_point = BasicPoint3d(0.0, 0.0, gps.ele)
        zone = 0
        is_north = False
        mgrs_code = ""

        try:
            geographiclib.utmups.Forward(gps.lat, gps.lon, zone, is_north, utm_point.x, utm_point.y)
            geographiclib.mgrs.Forward(zone, is_north, utm_point.x, utm_point.y, gps.lat, precision, mgrs_code)
        except geographiclib.GeographicErr as err:
            sys.stderr.write(err.what() + "\n")
            return mgrs_point

        mgrs_point.x = utm_point.x % 1e5
        mgrs_point.y = utm_point.y % 1e5
        self.projected_grid_ = mgrs_code

        if prev_projected_grid and prev_projected_grid != self.projected_grid_:
            sys.stderr.write("Projected MGRS Grid changed from last projection. Projected point "
                             "might be far away from previously projected point.\n"
                             "You may want to use a different projector.\n")

        return mgrs_point

    def reverse(self, mgrs_point: BasicPoint3d) -> GPSPoint:
        gps = GPSPoint(0.0, 0.0, 0.0)
        if self.isMGRSCodeSet():
            gps = self.reverse(mgrs_point, self.mgrs_code_)
        elif self.projected_grid_:
            gps = self.reverse(mgrs_point, self.projected_grid_)
        else:
            sys.stderr.write("Cannot run reverse operation if mgrs code is not set in projector.\n"
                             "Use setMGRSCode function or explicitly give mgrs code as an argument.\n")
        return gps

    def reverse(self, mgrs_point: BasicPoint3d, mgrs_code: str) -> GPSPoint:
        gps = GPSPoint(0.0, 0.0, mgrs_point.z)
        utm_point = BasicPoint3d(0., 0., gps.ele)

        zone = 0
        prec = 0
        is_north = False
        try:
            geographiclib.mgrs.Reverse(mgrs_code, zone, is_north, utm_point.x, utm_point.y, prec, False)
            utm_point.x += mgrs_point.x % math.pow(10, 5 - prec)
            utm_point.y += mgrs_point.y % math.pow(10, 5 - prec)
            geographiclib.utmups.Reverse(zone, is_north, utm_point.x, utm_point.y, gps.lat, gps.lon)
        except geographiclib.GeographicErr as err:
            sys.stderr.write("Failed to convert from MGRS to WGS\n")
            return gps

        return gps

    def setMGRSCode(self, mgrs_code: str):
        self.mgrs_code_ = mgrs_code

    def setMGRSCode(self, gps: GPSPoint, precision: int):
        utm_point = BasicPoint3d(0., 0., gps.ele)
        zone = 0
        is_north = False
        mgrs_code = ""

        try:
            geographiclib.utmups.Forward(gps.lat, gps.lon, zone, is_north, utm_point.x, utm_point.y)
            geographiclib.mgrs.Forward(zone, is_north, utm_point.x, utm_point.y, gps.lat, precision, mgrs_code)
        except geographiclib.GeographicErr as err:
            sys.stderr.write(err.what() + "\n")

        self.setMGRSCode(mgrs_code)



def main():
    pass

if __name__ == '__main__':
    main()