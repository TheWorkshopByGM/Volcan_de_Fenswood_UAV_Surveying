"""
Very simple image processor based on example from
https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
"""
import rclpy                                                    # type: ignore
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String

# ROS Topics
"""
/topic: [msg type] desciption

 UAV TOPICS: <<<
Subscribers:
/vehicle_1/camera/image_raw: [Image] Raw Image


 My Topics: <<<
Subscribers:
/vehicle_1/handshake: [String] Used for handshake

Publisher:
/vehicle_1/controller/img_handshake: [String] Used for handshake


"""

class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')
        self.br = CvBridge()
        self.img = None
        self.shp = [0,0]

    def start(self):
        # Subscribers
        self.img_sub = self.create_subscription(Image, '/vehicle_1/camera/image_raw', self.image_callback, 10) # set up subscriber for image
        self.handshake_sub = self.create_subscription(String, '/vehicle_1/handshake', self.handshake_callback, 10) # set up subscriber for Handshake from Controller
        
        # Publishers
        self.handshake_pub = self.create_publisher(String, '/vehicle_1/controller/img_handshake', 10) # create publisher for Controller Handshake

        # Timers
        self.timer = self.create_timer(5.0, self.timer_callback) # create a ROS2 timer to run the control actions

    
    def image_callback(self,msg): # on receiving image, convert and log information
        self.img = self.br.imgmsg_to_cv2(msg) # can do OpenCV stuff on img now
        self.shp = self.img.shape # just get the size

    def timer_callback(self):
        #self.get_logger().info('Got an image of {} x {}'.format(self.shp[0],self.shp[1]))
        return

    def handshake_callback(self,msg): # Handshake received, send handshake back
        self.get_logger().info('Received Handshake:  {}'.format(msg.data))
        if msg.data == 'Controller requesting handshake':
            self.handshake_msg = String()
            self.handshake_msg.data = 'Handshake from image processor'
            self.handshake_pub.publish(self.handshake_msg)
        else:
            self.get_logger().info('Wrong Handshake Msg!')
                    

def main(args=None):
    
    rclpy.init(args=args)

    image_node = ImageProcessor()
    image_node.start()
    rclpy.spin(image_node)


if __name__ == '__main__':
    main()