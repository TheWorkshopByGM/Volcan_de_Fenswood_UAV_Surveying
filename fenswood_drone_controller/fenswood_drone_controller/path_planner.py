import rclpy                                                    # type: ignore
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import os, sys
import cv2
from cv_bridge import CvBridge
import numpy as np

class PathPlanner(Node):

    def __init__(self):
        super().__init__('path_planner')
        #region Variables
        self.br = CvBridge()       # variable to allow for the transfer of ROS image to CVimage
        self.red_img = None
        self.yellow_img = None
        self.no_land_img = None
        self.red_img_received = False
        self.yellow_img_received = False
        self.red_counter = 0
        self.yellow_counter = 0
        self.closest_land_loc = None
        self.closest_land_loc_found = False
        #Tunable variable
        self.new_dimensions = (240,200) #Reshape/scale masks (Keep ratio of 6:5 | OG was 4:3)
        self.x_location = 325 #Mask location on map #Make sure that in new_dims(Y,X), X+x_loc < 823 (823: size of map)
        self.y_location = 540 #Mask location on map #Make sure that in new_dims(Y,X), Y+y_loc < 823 (823: size of map)
        #endregion

    def start(self):
        #region Subscribers
        self.handshake_sub = self.create_subscription(String, '/vehicle_1/handshake', self.handshake_callback, 10) # set up subscriber for Handshake from Controller
        self.red_img_sub = self.create_subscription(Image, '/vehicle_1/controller/red_img_persp',self.red_img_callback, 10) # create publisher for Red Img
        self.yellow_img_sub = self.create_subscription(Image, '/vehicle_1/controller/yellow_img_persp',self.yellow_img_callback, 10) # create publisher for Yellow Img
        self.interaction_command_sub = self.create_subscription(String, 'interaction_topic', self.interaction_command_callback, 10) #James_edit
        #endregion

        #region Publishers
        self.handshake_pub = self.create_publisher(String, '/vehicle_1/controller/pp_handshake', 10) # create publisher for Controller Handshake
        self.no_land_mask_pub = self.create_publisher(Image, '/no_land_mask', 10) # create publisher for no_land_mask
        self.no_land_mask_pub2 = self.create_publisher(Image, '/no_land_mask2', 10) # create publisher for no_land_mask
        self.no_land_mask_pub3 = self.create_publisher(Image, '/no_land_mask3', 10) # create publisher for no_land_mask
        self.no_land_mask_pub4 = self.create_publisher(Image, '/no_land_mask4', 10) # create publisher for no_land_mask
        self.no_land_mask_pub5 = self.create_publisher(Image, '/no_land_mask5', 10) # create publisher for no_land_mask
        self.no_fly_mask_pub = self.create_publisher(Image, '/no_fly_mask', 10) # create publisher for no_land_mask
        self.land_waypoints_pub1 = self.create_publisher(String, '/land_waypoints1', 10) # create publisher for no_land_mask
        self.land_waypoints_pub2 = self.create_publisher(String, '/land_waypoints2', 10) # create publisher for no_land_mask
        #endregion

    #Callbacks
    def handshake_callback(self,msg): # Handshake received, send handshake back
        self.get_logger().info('Received Handshake:  {}'.format(msg.data))
        if msg.data == 'Controller requesting handshake':
            self.handshake_msg = String()
            self.handshake_msg.data = 'Handshake from path planner'
            self.handshake_pub.publish(self.handshake_msg)
        else:
            self.get_logger().info('Wrong Handshake Msg!')

    def red_img_callback(self,msg):
        threshold = 0
        if self.red_img_received == False:
            self.red_img = msg
            self.red_counter = self.red_counter + 1
            if self.red_counter > threshold:
                self.red_img_received = True
        if self.yellow_img_received == True:
            self.generate_no_land_img()

    def yellow_img_callback(self,msg):
        threshold = 0
        if self.yellow_img_received == False:
            self.yellow_img = msg
            self.yellow_counter = self.yellow_counter + 1
            if self.yellow_counter > threshold:
                self.yellow_img_received = True
        if self.red_img_received == True:
            self.generate_no_land_img()

    def interaction_command_callback(self,msg):
        if (self.red_img_received == True) and (self.yellow_img_received == True):
            if msg.data == 'up':
                if (self.new_dimensions[0]+self.y_location)>5:
                    self.y_location = self.y_location-5
                    self.generate_no_land_img()
            if msg.data == 'down':
                if (self.new_dimensions[0]+self.y_location)<817:
                    self.y_location = self.y_location+5
                    self.generate_no_land_img()
            if msg.data == 'left':
                if (self.new_dimensions[1]+self.x_location)>5:
                    self.x_location = self.x_location-5
                    self.generate_no_land_img()
            if msg.data == 'right':
                if (self.new_dimensions[1]+self.x_location)<817:
                    self.x_location = self.x_location+5
                    self.generate_no_land_img()  

    #Functions
    def generate_no_land_img(self):
        #Merge red and yellow masks
        yellow_mask = self.br.imgmsg_to_cv2(self.yellow_img)
        red_mask_OG = self.br.imgmsg_to_cv2(self.red_img)
        self.no_land_img =  cv2.add(yellow_mask,red_mask_OG)
        #Crop Mask to desired Region of Interest
        self.no_land_img = self.no_land_img[0:300,60:420]

        #Scale image
        new_dims = self.new_dimensions # add to (y_loc,x_loc)
        self.no_land_img = cv2.resize(self.no_land_img, new_dims, interpolation= cv2.INTER_LINEAR)
        #Rotate CCW
        self.no_land_img=cv2.transpose(self.no_land_img)
        self.no_land_img=cv2.flip(self.no_land_img,flipCode=0)
        self.no_land_mask_pub.publish(self.br.cv2_to_imgmsg(self.no_land_img,encoding='bgr8'))

        #Gradually Scale image
        new_dims = (100,120)
        self.no_land_img = cv2.resize(self.no_land_img, new_dims, interpolation= cv2.INTER_NEAREST)
        #Increase White Contrast in Image
        self.no_land_img =  cv2.add(self.no_land_img ,self.no_land_img)
        self.no_land_img =  cv2.add(self.no_land_img ,self.no_land_img)
        #Scale Egain
        new_dims = (40,48)
        self.no_land_img = cv2.resize(self.no_land_img, new_dims, interpolation= cv2.INTER_NEAREST)
        #Increase White Contrast in Image Again
        self.no_land_img =  cv2.add(self.no_land_img ,self.no_land_img)
        self.no_land_img =  cv2.add(self.no_land_img ,self.no_land_img)
        #Scale Again
        new_dims = (20,24) 
        self.no_land_img = cv2.resize(self.no_land_img, new_dims, interpolation= cv2.INTER_NEAREST)
        #Increase White Contrast in Image Again
        self.no_land_img =  cv2.add(self.no_land_img ,self.no_land_img)
        self.no_land_img =  cv2.add(self.no_land_img ,self.no_land_img)
        self.no_land_img =  cv2.add(self.no_land_img ,self.no_land_img)
        self.no_land_img =  cv2.add(self.no_land_img ,self.no_land_img)
        self.no_land_img =  cv2.add(self.no_land_img ,self.no_land_img)
        self.no_land_img =  cv2.add(self.no_land_img ,self.no_land_img)
        self.no_land_mask_pub2.publish(self.br.cv2_to_imgmsg(self.no_land_img,encoding='bgr8'))
        #Dilate Image
        kernel = np.ones((4, 4), 'uint8')
        self.no_land_img = cv2.dilate(self.no_land_img, kernel, iterations=1)

        #Add Crater
        crater_colour = [0, 0, 255]
        crater_coordinates = (10,11)
        crater_radius = 8
        temp_img = cv2.add(self.no_land_img ,self.no_land_img)
        temp_img = cv2.circle(temp_img, crater_coordinates, crater_radius, crater_colour, -1)
        crater_colour = [0, 0, 0]
        crater_coordinates = (10,11)
        crater_radius = 7
        temp_img = cv2.circle(temp_img, crater_coordinates, crater_radius, crater_colour, -1)
        self.no_land_mask_pub3.publish(self.br.cv2_to_imgmsg(temp_img,encoding='bgr8'))
        self.no_land_mask_pub4.publish(self.br.cv2_to_imgmsg(self.no_land_img,encoding='bgr8'))
        self.no_land_img =  cv2.subtract(temp_img,self.no_land_img)
        #Remove Bottom pixels as outside of no land zone
        height, width, channels = self.no_land_img.shape
        for i in range(width):
            column = width - i -1
            for j in range(height):
                row = height - j - 1
                if row > 13:
                    self.no_land_img[row][column] = [0,0,0]
        #Publish Img for Troobleshooting
        self.no_land_mask_pub5.publish(self.br.cv2_to_imgmsg(self.no_land_img,encoding='bgr8'))
        #Find Landing Location
        self.find_land_loc()

    def find_land_loc(self):
        height, width, channels = self.no_land_img.shape
        for i in range(width):
            column = width - i -1
            for j in range(height):
                row = height - j - 1
                if self.no_land_img[row][column][2] == 255 and self.closest_land_loc_found == False:
                    self.closest_land_loc = (row,column)
                    self.closest_land_loc_found = True
        #self.get_logger().info('CLOSEST LANDING LOCATION IS: col:{}, row:{}, no_land_img[{}][{}]:{}'.format(self.closest_land_loc[1],self.closest_land_loc[0],self.closest_land_loc[0],self.closest_land_loc[1],self.no_land_img[self.closest_land_loc[0]][self.closest_land_loc[1]]))
        self.generate_no_fly_img()

    def generate_no_fly_img(self):
        #Get red masks
        self.no_fly_img = self.br.imgmsg_to_cv2(self.red_img)
        #Crop Mask to desired Region of Interest
        self.no_fly_img = self.no_fly_img[0:300,60:420]

        #Scale image
        new_dims = self.new_dimensions # add to (y_loc,x_loc)
        self.no_fly_img = cv2.resize(self.no_fly_img, new_dims, interpolation= cv2.INTER_LINEAR)
        #Rotate CCW
        self.no_fly_img=cv2.transpose(self.no_fly_img)
        self.no_fly_img=cv2.flip(self.no_fly_img,flipCode=0)

        #Gradually Scale image
        new_dims = (100,120)
        self.no_fly_img = cv2.resize(self.no_fly_img, new_dims, interpolation= cv2.INTER_NEAREST)
        #Increase White Contrast in Image
        self.no_fly_img =  cv2.add(self.no_fly_img ,self.no_fly_img)
        self.no_fly_img =  cv2.add(self.no_fly_img ,self.no_fly_img)
        #Scale Egain
        new_dims = (40,48)
        self.no_fly_img = cv2.resize(self.no_fly_img, new_dims, interpolation= cv2.INTER_NEAREST)
        #Increase White Contrast in Image Again
        self.no_fly_img =  cv2.add(self.no_fly_img ,self.no_fly_img)
        self.no_fly_img =  cv2.add(self.no_fly_img ,self.no_fly_img)
        #Scale Again
        new_dims = (20,24) 
        self.no_fly_img = cv2.resize(self.no_fly_img, new_dims, interpolation= cv2.INTER_NEAREST)
        #Increase White Contrast in Image Again
        self.no_fly_img =  cv2.add(self.no_fly_img ,self.no_fly_img)
        self.no_fly_img =  cv2.add(self.no_fly_img ,self.no_fly_img)
        self.no_fly_img =  cv2.add(self.no_fly_img ,self.no_fly_img)
        self.no_fly_img =  cv2.add(self.no_fly_img ,self.no_fly_img)
        self.no_fly_img =  cv2.add(self.no_fly_img ,self.no_fly_img)
        self.no_fly_img =  cv2.add(self.no_fly_img ,self.no_fly_img)
        self.no_fly_img =  cv2.add(self.no_fly_img ,self.no_fly_img)
        self.no_fly_img =  cv2.add(self.no_fly_img ,self.no_fly_img)
        #Dilate Image
        kernel = np.ones((4, 4), 'uint8')
        self.no_fly_img = cv2.dilate(self.no_fly_img, kernel, iterations=1)

        #Add White to Top and bottom pixels as they are no fly zones
        height, width, channels = self.no_fly_img.shape
        for i in range(width):
            column = width - i -1
            for j in range(height):
                row = height - j - 1
                if row < 2:
                    self.no_fly_img[row][column] = [255,255,255]
                if row >16:
                    self.no_fly_img[row][column] = [255,255,255]
        #Publish Img for Troobleshooting
        self.no_fly_mask_pub.publish(self.br.cv2_to_imgmsg(self.no_fly_img,encoding='bgr8'))
        #Find Landing Location
        self.find_path_to_land()

    def find_path_to_land(self):
        height, width, channels = self.no_land_img.shape
        #self.get_logger().info('height, width, channels = {}, {},{}'.format(height, width, channels))
        #Generate Matrix of same size as image with 255 everywhere. Will be updated by algorithm later
        path_map = np.full((height, width, 1), 255, np.uint8)
        #Set first column on right to 0's
        for j in range(height):
            row = height - j - 1
            #self.get_logger().info('self.no_fly_img[{}][{}][0]={}'.format(row,width-1,self.no_fly_img[row][width-1][0]))
            if self.no_fly_img[row][width-1][0] == 0:
                #self.get_logger().info('We are in mother F***ers!')
                path_map[row][width-1] = 0
        #self.get_logger().info('Path Map Last Column Modified:{}'.format(path_map))
        #Starting from right to left, build the distance matrix (Attention! My brain is overheating at this point! F***)    
        for i in range(width-1):
            column = width - i -2 #Eliminate rightmost columns already processed Bit**
            #Do the path first from bottom to top
            for j in range(height):
                row = height - j - 1
                #Check Vertical Edge Cases
                if row <= 0:
                    #self.get_logger().info('First Row Loop: Row = {}, if'.format(row))
                    top_px = 255
                    bottom_px = path_map[row+1][column]
                elif row >= height-2:
                    #self.get_logger().info('First Row Loop: Row = {}, elif'.format(row))
                    top_px = path_map[row-1][column]
                    bottom_px = 255
                else:
                    #self.get_logger().info('First Row Loop: Row = {}, else'.format(row))
                    top_px = path_map[row-1][column]
                    bottom_px = path_map[row+1][column]
                #Check Horizontal Edge Cases
                if column == 0:
                    #self.get_logger().info('First Row Loop: column = {}, if'.format(column))
                    left_px = 255
                    right_px = path_map[row][column+1]
                else:
                    #self.get_logger().info('First Row Loop: column = {}, else'.format(column))
                    left_px = path_map[row][column-1]
                    right_px = path_map[row][column+1]
                #Update Map
                #self.get_logger().info('Col:{},Row:{},Top,Bottom,Left,Right:{},{},{},{}'.format(column,row,top_px,bottom_px,left_px,right_px))
                if self.no_fly_img[row][column][0] == 0:
                    if path_map[row][column]>top_px:
                        #self.get_logger().info('path_map[{}][{}] updated to TOP:{}'.format(row,column,top_px+1))
                        path_map[row][column]=top_px+1
                    if path_map[row][column]>bottom_px:
                        #self.get_logger().info('path_map[{}][{}] updated to Bottom:{}'.format(row,column,bottom_px+1))
                        path_map[row][column]=bottom_px+1
                    if path_map[row][column]>left_px:
                        #self.get_logger().info('path_map[{}][{}] updated to left:{}'.format(row,column,left_px+1))
                        path_map[row][column]=left_px+1
                    if path_map[row][column]>right_px:
                        #self.get_logger().info('path_map[{}][{}] updated to right:{}'.format(row,column,right_px+1))
                        path_map[row][column]=right_px+1
            #Then do the path first from top to bottom
            for j in range(height):
                row = height -1
                #Check Vertical Edge Cases
                if row <= 0:
                    top_px = 255
                    bottom_px = path_map[row+1][column]
                elif row >= height-2:
                    top_px = path_map[row-1][column]
                    bottom_px = 255
                else:
                    top_px = path_map[row-1][column]
                    bottom_px = path_map[row+1][column]
                #Check Horizontal Edge Cases
                if column == 0:
                    left_px = 255
                    right_px = path_map[row][column+1]
                else:
                    left_px = path_map[row][column-1]
                    right_px = path_map[row][column+1]
                #Update Map
                if self.no_fly_img[row][column][0] == 0:
                    if path_map[row][column]>top_px:
                        path_map[row][column]=top_px+1
                    if path_map[row][column]>bottom_px:
                        path_map[row][column]=bottom_px+1
                    if path_map[row][column]>left_px:
                        path_map[row][column]=left_px+1
                    if path_map[row][column]>right_px:
                        path_map[row][column]=right_px+1
        #Print Matrix
        #self.get_logger().info('Path Map:{}'.format(path_map))

        #Now for the Second Part, lets find the FFFFF*****inggg path, Oh my god this is taking for EVERRRR! :(
        #self.closest_land_loc = (row,column)
        self.path_to_land = []
        self.path_to_land.append(self.closest_land_loc)
        current_loc_coord = self.closest_land_loc
        row = self.closest_land_loc[0]
        column = self.closest_land_loc[1]
        current_distance = path_map[row][column]
        #IF distance already 0 then done
        if column == width-1:
            path_found = True
        else:
            path_found = False
        #self.get_logger().info('Pre While Current distance: {}, path_found:{}'.format(current_distance,path_found))
        #self.get_logger().info('Pre While, path_to_land:{}'.format(self.path_to_land))
        #Get Current Pt row and columns
        row = self.closest_land_loc[0]
        column = self.closest_land_loc[1]
        while path_found == False:
            
            #Check Vertical Edge Cases
            if row <= 0:
                top_px = 255
                bottom_px = path_map[row+1][column]
            elif row >= height-1:
                top_px = path_map[row-1][column]
                bottom_px = 255
            else:
                top_px = path_map[row-1][column]
                bottom_px = path_map[row+1][column]
            #Check Horizontal Edge Cases
            if column == 0:
                left_px = 255
                right_px = path_map[row][column+1]
            elif column >= width-1:
                left_px = path_map[row][column-1]
                right_px = 255
            else:
                left_px = path_map[row][column-1]
                right_px = path_map[row][column+1]
            #self.get_logger().info('Col:{},Row:{},Current:{},Top,Bottom,Left,Right:{},{},{},{}'.format(column,row,current_distance,top_px,bottom_px,left_px,right_px))
            #Find Next Loc
            next_loc = current_loc_coord
            next_dist = current_distance
            if next_dist>top_px:
                next_dist = top_px
                next_loc = (row-1,column)
                #self.get_logger().info('Moving Up!')
            if next_dist>bottom_px:
                next_dist = bottom_px
                next_loc = (row+1,column)
                #self.get_logger().info('Moving Down!')
            if next_dist>left_px:
                next_dist = left_px
                next_loc = (row,column-1)
                #self.get_logger().info('Moving Left!')
            if next_dist>right_px:
                next_dist = right_px
                next_loc = (row,column+1)
                #self.get_logger().info('Moving Right!')
            
            self.path_to_land.append(next_loc)  
            #self.get_logger().info('In While, path_to_land:{}'.format(self.path_to_land))
            if  next_loc[1] == width-1:
                path_found = True
            row = next_loc[0]
            column = next_loc[1]
        #self.get_logger().info('PATH FOUND!')

        #Time to convert those pixel coordinates to GPS Locations!
        #UAV_X = int(((self.msg_array[2*x+1] + 2.671721)/(-2.665688 + 2.671721))*(800 - 57) + 57) # transform the latitude and longitude coordinates to pixel values
        #UAV_Y = int(((self.msg_array[2*x] - 51.424692)/(51.421246 - 51.424692))*(774 - 90) + 90)
        #GPS_Y = ((Ypx -57)/(800-57))*(-2.665688 + 2.671721)-2.671721
        #GPS_X = ((Xpx -90)/(774-90))*(51.421246 - 51.424692)+51.424692
        self.gps_waypoints = []
        waypoint_numbers = len(self.path_to_land)
        for i in range(waypoint_numbers):
            loc = self.path_to_land[waypoint_numbers-i-1]
            loc_y = self.x_location + loc[1]*10
            loc_x = self.y_location + loc[0]*10
            GPS_Y = ((loc_y -57)/(800-57))*(-2.665688 + 2.671721)-2.671721
            #self.get_logger().info('GPS_Y:{}'.format(GPS_Y))
            msg = String()
            msg.data = str(GPS_Y)
            self.land_waypoints_pub1.publish(msg)
            GPS_X = ((loc_x -90)/(774-90))*(51.421246 - 51.424692)+51.424692
            #self.get_logger().info('GPS_X:{}'.format(GPS_X))
            msg = String()
            msg.data = str(GPS_X)
            self.land_waypoints_pub2.publish(msg)
        #self.get_logger().info('Done Sending GPS Coord!')
        msg = String()
        msg.data = str(999)
        self.land_waypoints_pub1.publish(msg)
        self.land_waypoints_pub2.publish(msg)




def main(args=None):
    
    rclpy.init(args=args)

    path_planner = PathPlanner()
    path_planner.start()
    rclpy.spin(path_planner)

if __name__ == '__main__':
    main()