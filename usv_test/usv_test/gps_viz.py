import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray

from math import cos, pi


class GpsViz(Node):

    def __init__(self):
        super().__init__('gps_viz')

        self.sub = self.create_subscription(NavSatFix, '/fix', self.fix_cb, 10)

        self.pub = self.create_publisher(Float64MultiArray, '/gps/metres', 10)

        self.datum = None


    def fix_cb(self, msg):
        lat = msg.latitude
        lon = msg.longitude

        lat_m, lon_m = self.lat_lon_to_m(lat,lon)

        if self.datum == None:
            self.datum = [lat_m, lon_m]

        lat_m -= self.datum[0]
        lon_m -= self.datum[1]

        
        new_data = Float64MultiArray()
        new_data.data = [lat_m, lon_m]
        self.pub.publish(new_data)


    def lat_lon_to_m(self, lat, lon):

        lat_rad = lat * pi/180

        lat_m = lat * (111132.92 - 559.82*cos(2*lat_rad) + 1.175*cos(4*lat_rad) - 0.0023*cos(6*lat_rad))

        lon_m = lon * (111412.84*cos(lat_rad) - 93.5*cos(3*lat_rad) + 0.118*cos(5*lat_rad))

        return lat_m, lon_m

def main(args=None):
    rclpy.init(args=args)
    gps_viz = GpsViz()
    rclpy.spin(gps_viz)

if __name__ == '__main__':
    main()