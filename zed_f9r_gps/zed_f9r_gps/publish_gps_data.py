import rclpy
import serial
from rclpy.node import Node
from ublox_gps import UbloxGps
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class GPSPublisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.port = serial.Serial('/dev/serial0', baudrate=38400, timeout=1)
        self.gps = UbloxGps(self.port)

    def publish(self):
        try: 
            print("Listening for UBX Messages")
            while True:
                try:
                    geo = self.gps.geo_coords()
                    cov = self.gps.geo_cov()
                    msg = NavSatFix()
                    msg.longitude = geo.lon
                    msg.latitude = geo.lat
                    msg.altitude = 0.001 * float(geo.height)
                    NN = cov.posCovNN
                    NE = cov.posCovNE
                    ND = cov.posCovND
                    EE = cov.posCovEE
                    ED = cov.posCovED
                    DD = cov.posCovDD
                    # Following the convention here: https://www.ros.org/reps/rep-0105.html
                    msg.position_covariance = [EE, NE, ED, NE, NN, ND, ED, ND, DD]
                    self.publisher_.publish(msg)
                except (ValueError, IOError) as err:
                    print(err)
        finally:
            port.close()


def main(args=None):
    rclpy.init(args=args)

    gps_publisher = GPSPublisher()
    gps_publisher.publish()

    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

