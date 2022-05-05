import rclpy                                                    # type: ignore
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
import numpy as np

class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')
        # region Variables
        self.br = CvBridge()
        self.img_pro_req_bool = False #Bool to store if image processing has been requested
        #endregion

    def start(self):
        # region Subscribers
        self.img_sub = self.create_subscription(Image, '/vehicle_1/camera/image_raw', self.image_callback, 10) # set up subscriber for image
        self.handshake_sub = self.create_subscription(String, '/vehicle_1/handshake', self.handshake_callback, 10) # set up subscriber for Handshake from Controller
        self.img_pro_req_sub = self.create_subscription(String, '/vehicle_1/img_pro_req', self.img_pro_req_callback, 10) # set up subscriber for Image Processing Request from Controller
        #endregion

        # region Publishers  
        self.handshake_pub = self.create_publisher(String, '/vehicle_1/controller/img_handshake', 10) # create publisher for Controller Handshake
        self.red_img_pub = self.create_publisher(Image, '/vehicle_1/controller/red_img', 10) # create publisher for Red Img
        self.yellow_img_pub = self.create_publisher(Image, '/vehicle_1/controller/yellow_img', 10) # create publisher for Yellow Img
        self.red_img_persp_pub = self.create_publisher(Image, '/vehicle_1/controller/red_img_persp', 10) # create publisher for Red Img
        self.yellow_img_persp_pub = self.create_publisher(Image, '/vehicle_1/controller/yellow_img_persp', 10) # create publisher for Yellow Img
        #endregion

    def handshake_callback(self,msg): # Handshake received, send handshake back
        self.get_logger().info('Received Handshake:  {}'.format(msg.data))
        if msg.data == 'Controller requesting handshake':
            self.handshake_msg = String()
            self.handshake_msg.data = 'Handshake from image processor'
            self.handshake_pub.publish(self.handshake_msg)
        else:
            self.get_logger().info('Wrong Handshake Msg!')
     
    def img_pro_req_callback(self,msg): # Handshake received, send handshake back
        if msg.data == 'Controller requesting Image Processing':
            self.get_logger().info('IMAGE PROCESSING REQUESTED!')
            self.img_pro_req_bool = True
        else:
            self.get_logger().info('Wrong Request Msg!')

    def image_callback(self,msg): # on receiving image, convert and log information
        if self.img_pro_req_bool == True:
            self.img = self.br.imgmsg_to_cv2(msg) # can do OpenCV stuff on img now
            image_hsv = cv2.cvtColor(self.img, cv2.COLOR_RGB2HSV)
            # Red Mask
            mask1 = cv2.inRange(image_hsv, (0, 70, 50), (10, 255, 255))
            mask2 = cv2.inRange(image_hsv, (220, 70, 50), (255, 255, 255))
            red_mask = cv2.bitwise_or(mask1, mask2)
            red_mask_bgr = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2RGB)   
            self.red_img = self.br.cv2_to_imgmsg(red_mask_bgr,encoding='bgr8')
            self.red_img_pub.publish(self.red_img)

            # Yellow Mask
            yellow_mask = cv2.inRange(image_hsv, (25,70,50), (35, 255, 255))
            yellow_mask_bgr = cv2.cvtColor(yellow_mask, cv2.COLOR_GRAY2RGB)
            self.yellow_img = self.br.cv2_to_imgmsg(yellow_mask_bgr,encoding='bgr8')
            self.yellow_img_pub.publish(self.yellow_img)    

            #Perspective 640 × 480
            #specifying the points in the source image which is to be transformed to the corresponding points in the destination image
            taper = 50
            srcpts = np.float32([[0, 0], [0, 480], [640, 480], [640, 0]])
            destpts = np.float32([[0, 0], [0+taper, 480], [640-taper, 480], [640, 0]])
            #applying PerspectiveTransform() function to transform the perspective of the given source image to the corresponding points in the destination image
            resmatrix = cv2.getPerspectiveTransform(srcpts, destpts)
            #applying warpPerspective() function to display the transformed image
            red_resultimage = cv2.warpPerspective(red_mask_bgr, resmatrix, (640, 480)) 
            yellow_resultimage = cv2.warpPerspective(yellow_mask_bgr, resmatrix, (640, 480))  
            self.red_img_persp = self.br.cv2_to_imgmsg(red_resultimage,encoding='bgr8')
            self.yellow_img_persp = self.br.cv2_to_imgmsg(yellow_resultimage,encoding='bgr8')
            self.red_img_persp_pub.publish(self.red_img_persp)   
            self.yellow_img_persp_pub.publish(self.yellow_img_persp)   

def main(args=None):
    
    rclpy.init(args=args)

    image_node = ImageProcessor()
    image_node.start()
    rclpy.spin(image_node)

if __name__ == '__main__':
    main()