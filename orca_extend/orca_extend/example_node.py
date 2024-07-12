import rclpy
from rclpy.node import Node
from std_msgs import CameraInfo
from geometry_msgs import Point, Pose, PoseStamped
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from pyzbar import pyzbar

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        self.publisher_ = self.create_publisher(String, 'example_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.rightCamSub = self.create_subscription(CameraInfo,
            '/stereo_right/camera_info',
            self.handle_right_cam_msg,
            10)
        self.posSub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.handle_loc_msg, 10)
        self.pos = (0,0,0)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
    
    def handle_loc_msg(self, msg):
        self.pos = (msg.Pose.x, msg.Pose.y, msg.Pose.z)


    def handle_right_cam_msg(self, msg):
          bridge = CvBridge()
          cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
          barcodes = pyzbar.decode(cv_image)
          print(type(barcodes))
          # loop over the detected barcodes
          for barcode in barcodes:
            # extract the bounding box location of the barcode and draw the
            # bounding box surrounding the barcode on the image
            (x, y, w, h) = barcode.rect
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # the barcode data is a bytes object so if we want to draw it on
            # our output image we need to convert it to a string first
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            # draw the barcode data and barcode type on the image
            text = "{} ({})".format(barcodeData, barcodeType)
            cv2.putText(cv_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 255), 2)
            # print the barcode type and data to the terminal
            print("[INFO] Found {} barcode: {} at {}{}{}".format(barcodeType, barcodeData, self.pos.x, self.pos.y, self.pos.z))




def main(args=None):
    rclpy.init(args=args)

    QrDetector = ExampleNode()

    rclpy.spin(QrDetector)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
