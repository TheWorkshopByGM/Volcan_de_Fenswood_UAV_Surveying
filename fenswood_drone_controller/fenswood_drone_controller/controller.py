import rclpy
from rclpy.node import Node

# import message definitions for receiving status and position
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
# import message definition for sending setpoint
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import String
# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong

# import Extra packages
import numpy as np
import time

class FenswoodDroneController(Node):

    def __init__(self):
        super().__init__('controller')

        # Fixed Waypoints
        self.stage_1_waypoints_latlong = np.array([( 51.4233628,  -2.671671), (51.4237490, -2.6673513), (51.422820, -2.666802), (51.422198, -2.668096)])
        self.stage_1_waypoints_hdg = np.array([25, 115, 205, 245])

        # region Variables
        self.last_target = GeoPoseStamped() # and make a placeholder for the last sent target
        self.last_status = None     # store for last received status message
        self.last_pos = None       # store for last received position message
        self.init_alt = None       # store for global altitude at start
        self.last_alt_rel = None   # store for last altitude relative to start
        self.control_state = 'init' # initial state for finite state machine 
        self.error = 'all_good' # initial state for finite state machine 
        self.state_timer = 0 # timer for time spent in each state
        self.img_handshake = False # Boolean for Handshake with image processor
        self.ie_handshake = False # Boolean for Handshake with interface engine
        self.pp_handshake = False # Boolean for Handshake with path planner
        self.waypoint_index = 0 # Store Index of next Waypoint
        self.interaction_command = None 
        self.follow_path_home_initiated = False #Follow_Path_Home Initializer that subtracts 1 from waypoint index
        self.heading = None #Store Current Drone Heading
        self.desired_altitude = None #Store Desired Altitude
        self.GPS_Y = None
        self.land_waypoints = [(51.422198, -2.668096)]
        self.land_waypoints_received_1 = False
        self.land_waypoints_received_2 = False
        self.waypoint_index_to_land = 0 # Store Index of next Waypoint
        #endregion

        # region Clients
        self.cmd_cli = self.create_client(CommandLong, '/vehicle_1/mavros/cmd/command')# create service clients for long command (datastream requests)...
        self.mode_cli = self.create_client(SetMode, '/vehicle_1/mavros/set_mode') # ... for mode changes ...
        self.arm_cli = self.create_client(CommandBool, '/vehicle_1/mavros/cmd/arming')# ... for arming ...
        self.takeoff_cli = self.create_client(CommandTOL, '/vehicle_1/mavros/cmd/takeoff')# ... and for takeoff
        self.land_cli = self.create_client(CommandTOL, '/vehicle_1/mavros/cmd/land')# ... and for takeoff
        #endregion

        # region Publishers
        self.target_pub = self.create_publisher(GeoPoseStamped, '/vehicle_1/mavros/setpoint_position/global', 10) # create publisher for setpoint
        self.gimbal_pub = self.create_publisher(Float32, '/vehicle_1/gimbal_tilt_cmd', 10) #create publisher for gimbal controller
        self.img_handshake_pub = self.create_publisher(String, '/vehicle_1/handshake', 10) # create publisher for Image Processor Handshake
        self.img_pro_req_pub = self.create_publisher(String, '/vehicle_1/img_pro_req', 10) # create publisher for Image Processor Handshake
        self.ie_waypoints_pub = self.create_publisher(String, '/vehicle_1/ie/waypoints_S1', 10) # create publisher for Image Processor Handshake
        self.ie_waypoints_pub2 = self.create_publisher(String, '/vehicle_1/ie/waypoints_S2', 10) # create publisher for Image Processor Handshake
        self.state_pub = self.create_publisher(String, '/FSM_State', 10) # create publisher for the interface engine state 
        self.error_msg_pub = self.create_publisher(String, '/Error_msg', 10) # create publisher for the interface engine state 
        self.setpoint_velocity_pub = self.create_publisher(Twist, '/vehicle_1/mavros/setpoint_velocity/cmd_vel_unstamped', 10) # create publisher for the interface engine state #James_edit
        self.arming_status_pub = self.create_publisher(String, '/vehicle_1/armed', 10) # create publisher for the interface engine state 
        
        # endregion

    def start(self):
        
        # region Subscribers
        self.state_sub = self.create_subscription(State, '/vehicle_1/mavros/state', self.state_callback, 10) # set up two subscribers, one for vehicle state...
        self.pos_sub = self.create_subscription(NavSatFix, '/vehicle_1/mavros/global_position/global', self.position_callback, 10) # ...and the other for global position
        self.head_sub = self.create_subscription(Float64, '/vehicle_1/mavros/global_position/compass_hdg', self.heading_callback, 10) # ...and the other for compass heading
        self.img_handshake_sub = self.create_subscription(String, '/vehicle_1/controller/img_handshake', self.img_handshake_callback, 10) # set up subscriber for Image Image Processor Handshake
        self.ie_handshake_sub = self.create_subscription(String, '/vehicle_1/controller/ie_handshake', self.ie_handshake_callback, 10) # set up subscriber for Image Image Processor Handshake
        self.pp_handshake_sub = self.create_subscription(String, '/vehicle_1/controller/pp_handshake', self.pp_handshake_callback, 10) # set up subscriber for Image Image
        self.interaction_command_sub = self.create_subscription(String, 'interaction_topic', self.interaction_command_callback, 10) 
        self.land_waypoints_sub1 = self.create_subscription(String, '/land_waypoints1',self.land_waypoints_callback1, 10) # create publisher for no_land_mask
        self.land_waypoints_sub2 = self.create_subscription(String, '/land_waypoints2',self.land_waypoints_callback2, 10) # create publisher for no_land_mask
        #endregion 
        
        # region Timers
        self.timer = self.create_timer(1.0, self.timer_callback) # create a ROS2 timer to run the control actions
        #endregion 

    ### CALLBACKS ###
    def interaction_command_callback(self,msg): 
        self.interaction_command = msg.data #none,launch #James_edit

    def state_callback(self,msg): # on receiving status message, save it to global 
        self.last_status = msg 
        self.get_logger().debug('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))

    def position_callback(self,msg): # on receiving positon message, save it to global
        #self.get_logger().info('Drone at {}N,{}E altitude {}m'.format(msg.latitude,msg.longitude,msg.altitude))
        if self.init_alt: # determine altitude relative to start
            self.last_alt_rel = msg.altitude - self.init_alt
        self.last_pos = msg
        self.get_logger().info('Drone at {}N,{}E altitude {} m'.format(msg.latitude,msg.longitude,self.last_alt_rel))

    def heading_callback(self,msg): # on receiving positon message, save it to global
        self.heading = msg.data
        self.get_logger().info('Drone heading {}'.format(self.heading))

    def img_handshake_callback(self,msg):
        if msg.data == 'Handshake from image processor':
            self.img_handshake = True
        else:
            self.get_logger().info('Wrong Handshake Msg from Image Processor!')

    def ie_handshake_callback(self,msg):
        if msg.data == 'Handshake from interface engine':
            self.ie_handshake = True
        else:
            self.get_logger().info('Wrong Handshake Msg from Interface Engine!')

    def pp_handshake_callback(self,msg):
        if msg.data == 'Handshake from path planner':
            self.pp_handshake = True
        else:
            self.get_logger().info('Wrong Handshake Msg from Path Planner!')

    def land_waypoints_callback1(self,msg):
        if  self.land_waypoints_received_1 == False:  
            self.get_logger().info('land_waypoints_callback1 received a msg!')
            if float(msg.data) == 999:
                self.land_waypoints_received_1 = True
                if self.land_waypoints_received_2 == True:
                    self.send_waypoints_2()
                    self.get_logger().info('Will Send Waypoints from callback1')
            else:    
                self.GPS_Y = float(msg.data)
                self.get_logger().info('Received Y waypoint from pp: {}'.format(self.GPS_Y))

    def land_waypoints_callback2(self,msg):
        if  self.land_waypoints_received_2 == False:    
            self.get_logger().info('land_waypoints_callback2 received a msg!')
            if float(msg.data) == 999:
                self.land_waypoints_received_2 = True
                if self.land_waypoints_received_1 == True:
                    self.get_logger().info('Will Send Waypoints from callback2')
                    self.send_waypoints_2()
            else:
                GPS = (float(msg.data),self.GPS_Y)
                self.get_logger().info('Received X waypoint from pp: {}'.format(float(msg.data)))
                self.land_waypoints.append(GPS)
                self.get_logger().info('Appended Coordinates, new waypoints look like this: {}'.format(self.land_waypoints))
                
    ### FUNCTIONS ###
    def request_data_stream(self,msg_id,msg_interval):
        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(msg_id)
        cmd_req.param2 = float(msg_interval)
        self.future = self.cmd_cli.call_async(cmd_req)
        self.get_logger().info('Requested msg {} every {} us'.format(msg_id,msg_interval))

    def change_mode(self,new_mode):
        mode_req = SetMode.Request()
        mode_req.custom_mode = new_mode
        self.future = self.mode_cli.call_async(mode_req)
        self.get_logger().info('Request sent for {} mode.'.format(new_mode))

    def arm_request(self):
        arm_req = CommandBool.Request()
        arm_req.value = True
        self.future = self.arm_cli.call_async(arm_req)
        self.get_logger().info('Arm request sent')

    def takeoff(self,target_alt):
        takeoff_req = CommandTOL.Request()
        takeoff_req.latitude = self.last_pos.latitude
        takeoff_req.longitude = self.last_pos.longitude
        takeoff_req.altitude = target_alt
        self.future = self.takeoff_cli.call_async(takeoff_req)
        self.get_logger().info('Requested takeoff to {}m'.format(target_alt))

    def land(self):
        land_req = CommandTOL.Request()
        land_req.latitude = self.last_pos.latitude
        land_req.longitude = self.last_pos.longitude
        self.future = self.land_cli.call_async(land_req)
        self.get_logger().info('Requested Landing')

    def flyto(self,lat,lon,alt):
        self.last_target.pose.position.latitude = lat
        self.last_target.pose.position.longitude = lon
        self.last_target.pose.position.altitude = alt
        self.target_pub.publish(self.last_target)
        self.get_logger().info('Sent drone to {}N, {}E, altitude {}m'.format(lat,lon,alt)) 

    def move_camera(self,angle):
        self.gimbal_msg = Float32()
        self.gimbal_msg.data = (angle/90 * 1.57)
        self.gimbal_pub.publish(self.gimbal_msg)
        self.get_logger().info('Moved Camera to {}° angle'.format(self.gimbal_msg.data))

    def distance_to_last(self):
        d_long = self.last_pos.longitude - self.last_target.pose.position.longitude
        d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
        distance = ((d_long)**2+(d_lat)**2)**0.5
        self.get_logger().info('Distance is {}'.format(distance)) 
        return distance

    def send_handshakes(self):
        self.handshake_msg = String()
        self.handshake_msg.data = 'Controller requesting handshake'
        self.img_handshake_pub.publish(self.handshake_msg)
        self.get_logger().info('Handshake Sent to Image Processor!')

    def req_img_processing(self):
        self.img_pro_req_msg = String()
        self.img_pro_req_msg.data = 'Controller requesting Image Processing'
        self.img_pro_req_pub.publish(self.img_pro_req_msg)
        self.get_logger().info('Image Processing Request Sent to Image Processor!')

    def send_state(self): #James_edit
        self.control_state_msg = String() #James_edit
        self.control_state_msg.data = self.control_state #James_edit
        self.state_pub.publish(self.control_state_msg) #James_edit
        #self.get_logger().info('Sent state!') #James_edit

    def send_error(self): #James_edit
        self.error_msg = String() #James_edit
        self.error_msg.data = self.error #James_edit
        self.error_msg_pub.publish(self.error_msg) #James_edit
        #self.get_logger().info('Sent error!') #James_edit

    def send_waypoints(self):
        # Publish Waypoints to IE
        self.waypoint_msg = String()
        self.waypoint_msg.data = str(self.stage_1_waypoints_latlong[0][0]) + ',' + str(self.stage_1_waypoints_latlong[0][1]) 
        for x in range(self.stage_1_waypoints_latlong.shape[0]-1):
            self.waypoint_msg.data = self.waypoint_msg.data + ',' + str(self.stage_1_waypoints_latlong[x+1][0]) + ',' + str(self.stage_1_waypoints_latlong[x+1][1]) 
        self.get_logger().info('waypoint_msg: {}'.format(self.waypoint_msg.data))
        self.ie_waypoints_pub.publish(self.waypoint_msg)
     
    def send_waypoints_2(self):
        # Publish Waypoints to IE
        self.waypoint_msg = String()
        self.waypoint_msg.data = str(self.land_waypoints[0][0]) + ',' + str(self.land_waypoints[0][1]) 
        for x in range(len(self.land_waypoints)-1):
            self.waypoint_msg.data = self.waypoint_msg.data + ',' + str(self.land_waypoints[x+1][0]) + ',' + str(self.land_waypoints[x+1][1]) 
        self.get_logger().info('waypoint2_msg: {}'.format(self.waypoint_msg.data))
        self.ie_waypoints_pub2.publish(self.waypoint_msg)
        
    def state_transition(self):
        if self.control_state =='init':
            if self.last_status:
                if self.last_status.system_status==3:
                    self.get_logger().info('Drone initialized')
                    self.request_data_stream(33, 1000000) # send command to request regular position updates
                    self.change_mode("GUIDED")  # change mode to GUIDED
                    return('handshake') # move on to arming
                else:
                    return('init')
            else:
                return('init')

        elif self.control_state == 'handshake':
            self.get_logger().info('Handshake Status: img:{}, ie:{}, pp:{}'.format(self.img_handshake,self.ie_handshake,self.pp_handshake))
            if self.img_handshake and self.ie_handshake and self.pp_handshake: # Handshake from image received
                self.get_logger().info('Hanshake successful')
                self.send_waypoints()
                self.error = 'all_good'
                self.send_error()
                return('arming')
            elif self.state_timer > 120: # timeout
                self.get_logger().error('Handshake Failed')
                return('exit')
            else:
                self.send_handshakes()
                return('handshake')
            
        elif self.control_state == 'arming':
            if self.last_status.armed:
                self.get_logger().info('Arming successful')
                if self.last_pos: # armed - grab init alt for relative working
                    self.last_alt_rel = 0.0
                    self.init_alt = self.last_pos.altitude
                return('takeoff_wait')
            elif self.state_timer > 500: # timeout
                self.get_logger().error('Failed to arm')
                self.error = 'timeout'#James_edit
                return('exit')#James_edit
            else:
                self.arm_request()
                self.error = 'all_good'
                self.send_error()
                return('arming')

        elif self.control_state == 'takeoff_wait': #James_edit
            if self.interaction_command == 'launch': #James_edit
                if self.last_status.armed:
                    self.desired_altitude = 60.0
                    my_msg = String()
                    my_msg.data = 'yes'
                    self.arming_status_pub.publish(my_msg)
                    self.takeoff(self.desired_altitude) # send takeoff command #James_edit
                    self.get_logger().info('Climbing!') #James_edit
                    return('climbing') #James_edit
                else:
                    self.arm_request()
                    return('takeoff_wait')
            elif self.interaction_command == 'abort': #James_edit command being sent by the user
                self.error = 'user_aborted' #James_edit #error to be sent to the interface engine
                return('exit') #James_edit
            elif self.state_timer > 2500: # timeout #James_edit
                self.get_logger().error('Failed to takeoff') #James_edit
                self.error = 'timeout'#James_edit
                return('exit') #James_edit
            else:
                self.get_logger().info('Waiting for takeoff command')
                return('takeoff_wait')

        elif self.control_state == 'climbing':
            if self.last_alt_rel > 29.0:
                self.get_logger().info('Close enough to flight altitude') # move drone by sending setpoint message
                self.move_camera(60)
                return('follow_path')
            elif self.interaction_command == 'abort':#James_edit
                self.error = 'user_aborted'#James_edit
                return('landing')#James_edit
            elif self.state_timer > 60: # timeout
                self.get_logger().error('Failed to reach altitude')
                self.error = 'Failed_target_height'#James_edit
                return('landing')
            else:
                self.get_logger().info('Climbing, altitude {}m'.format(self.last_alt_rel))
                return('climbing')

        elif self.control_state == 'follow_path':
            if self.waypoint_index == 0:
                if self.stage_1_waypoints_latlong.shape[0] == 0:
                    return('landing')
                else:
                    self.get_logger().info('Fly to:{},{}'.format(self.stage_1_waypoints_latlong[self.waypoint_index][0],self.stage_1_waypoints_latlong[self.waypoint_index][1]))
                    self.flyto(self.stage_1_waypoints_latlong[self.waypoint_index][0], self.stage_1_waypoints_latlong[self.waypoint_index][1], self.init_alt + self.desired_altitude - 47.0) # unexplained correction factor on altitude
                    return('on_way')
            elif self.interaction_command == 'abort':
                self.error = 'user_aborted'
                self.waypoint_index = self.waypoint_index -1
                return('follow_path_home2')
            elif self.waypoint_index == self.stage_1_waypoints_latlong.shape[0]:
                self.get_logger().info('Follow Path reached END OF PATH!')
                self.move_camera(81)
                return('look_at_volcano')
            else:
                self.get_logger().info('Fly to:{},{}'.format(self.stage_1_waypoints_latlong[self.waypoint_index][0],self.stage_1_waypoints_latlong[self.waypoint_index][1]))
                self.flyto(self.stage_1_waypoints_latlong[self.waypoint_index][0], self.stage_1_waypoints_latlong[self.waypoint_index][1], self.init_alt + self.desired_altitude - 47.0) # unexplained correction factor on altitude
                return('on_way')          

        elif self.control_state == 'on_way':
            if self.interaction_command == 'abort':
                self.error = 'user_aborted'
                return('follow_path_home2')
            distance = self.distance_to_last()
            if abs(distance) < 0.00001:
                self.get_logger().info('Close enough to target distance={}'.format(distance))
                self.desired_head = self.stage_1_waypoints_hdg[self.waypoint_index]
                self.waypoint_index = self.waypoint_index+1
                return('follow_path')
                #return('yaw_to_angle')
            elif self.state_timer > 250: # timeout
                self.get_logger().error('Failed to reach target')
                self.error = 'Failed_target_waypoint'#James_edit
                return('follow_path_home')
            else:
                #self.get_logger().info('Target error {}'.format(distance))
                return('on_way')

        elif self.control_state == 'yaw_to_angle':
            #self.desired_head = angle
            self.head_error = self.desired_head - self.heading
            if self.head_error < -5:
                self.get_logger().info('Error {} < -5'.format(self.head_error))
                self.setpoint_vel_msg = Twist()
                self.setpoint_vel_msg.angular.z = 0.2
                self.setpoint_velocity_pub.publish(self.setpoint_vel_msg)
                return('yaw_to_angle')
            elif self.head_error > 5:
                self.get_logger().info('Error {} > 5'.format(self.head_error))
                self.setpoint_vel_msg = Twist()
                self.setpoint_vel_msg.angular.z = -0.2
                self.setpoint_velocity_pub.publish(self.setpoint_vel_msg)
                return('yaw_to_angle')
            else:
                self.get_logger().info(' -5< Error {} < 5'.format(self.head_error))
                self.setpoint_vel_msg = Twist()
                self.setpoint_vel_msg.angular.z = 0.0
                self.setpoint_velocity_pub.publish(self.setpoint_vel_msg)
                return('follow_path')

        elif self.control_state == 'look_at_volcano':
            if self.interaction_command == 'abort':
                self.error = 'user_aborted'
                self.waypoint_index = self.waypoint_index -1
                return('follow_path_home2')
            self.desired_head = 270
            self.head_error = self.desired_head - self.heading
            if self.head_error < -5:
                self.get_logger().info('Error {} < -5'.format(self.head_error))
                self.setpoint_vel_msg = Twist()
                self.setpoint_vel_msg.angular.z = 0.1
                self.setpoint_velocity_pub.publish(self.setpoint_vel_msg)
                return('look_at_volcano')
            elif self.head_error > 5:
                self.get_logger().info('Error {} > 5'.format(self.head_error))
                self.setpoint_vel_msg = Twist()
                self.setpoint_vel_msg.angular.z = -0.1
                self.setpoint_velocity_pub.publish(self.setpoint_vel_msg)
                return('look_at_volcano')
            else:
                self.get_logger().info(' -5< Error {} < 5'.format(self.head_error))
                self.setpoint_vel_msg = Twist()
                self.setpoint_vel_msg.angular.z = 0.0
                self.setpoint_velocity_pub.publish(self.setpoint_vel_msg)
                time.sleep(5)
                return('img_processing')

        elif self.control_state == 'img_processing':
            if self.interaction_command == 'abort':
                self.error = 'user_aborted'
                self.waypoint_index = self.waypoint_index -1
                return('follow_path_home2')
            if (self.land_waypoints_received_1 == True) and (self.land_waypoints_received_2 == True):
                return('follow_path_to_land')
            else:
                self.req_img_processing()
                return('img_processing')

        elif self.control_state == 'follow_path_to_land':
            self.get_logger().info('Will follow path to land!')
            self.desired_altitude = 25
            if self.waypoint_index_to_land == 0:
                self.get_logger().info('if self.waypoint_index_to_land == 0!')
                if len(self.land_waypoints) == 0:
                    self.get_logger().info('if len(self.land_waypoints) == 0:')
                    self.waypoint_index = self.waypoint_index -1
                    return('follow_path_home2')
                else:
                    self.get_logger().info('Else Fly to:{},{}'.format(self.land_waypoints[self.waypoint_index_to_land][0],self.land_waypoints[self.waypoint_index_to_land][1]))
                    self.flyto(self.land_waypoints[self.waypoint_index_to_land][0], self.land_waypoints[self.waypoint_index_to_land][1], self.init_alt + self.desired_altitude - 47.0) # unexplained correction factor on altitude
                    return('on_way_to_land')
            elif self.interaction_command == 'abort':#James_edit
                self.get_logger().info('elif self.interaction_command == abort!')
                self.error = 'user_aborted'#James_edit
                return('follow_path_home')#James_edit
            elif self.waypoint_index_to_land == len(self.land_waypoints):
                self.get_logger().info('Follow Path to land reached END OF PATH!')
                self.waypoint_index_to_land = self.waypoint_index_to_land-1
                self.flyto(self.land_waypoints[self.waypoint_index_to_land][0], self.land_waypoints[self.waypoint_index_to_land][1], self.init_alt - 47.0)
                return('landing_on_crater')
            else:
                self.get_logger().info('follow path to land Fly to:{},{}'.format(self.land_waypoints[self.waypoint_index_to_land][0],self.land_waypoints[self.waypoint_index_to_land][1]))
                self.flyto(self.land_waypoints[self.waypoint_index_to_land][0], self.land_waypoints[self.waypoint_index_to_land][1], self.init_alt + self.desired_altitude - 47.0) # unexplained correction factor on altitude
                return('on_way_to_land')          

        elif self.control_state == 'on_way_to_land':
            if self.interaction_command == 'abort':
                self.error = 'user_aborted'
                return('follow_path_home')
            self.get_logger().info('On Way to Land!')
            distance = self.distance_to_last()
            if abs(distance) < 0.00001:
                self.get_logger().info('Close enough to target land distance={}'.format(distance))
                self.waypoint_index_to_land = self.waypoint_index_to_land+1
                return('follow_path_to_land')
            elif self.state_timer > 250: # timeout
                self.get_logger().error('Failed to reach land target')
                self.error = 'Failed_target_waypoint'
                return('follow_path_home')
            else:
                #self.get_logger().info('Target error {}'.format(distance))
                return('on_way_to_land')
        
        elif self.control_state == 'landing_on_crater':
            if self.interaction_command == 'abort':
                self.error = 'user_aborted'
                return('follow_path_home')
            self.get_logger().info('Landing!')
            if self.last_alt_rel < 5.0:
                self.get_logger().info('Close enough to land altitude={}'.format(self.last_alt_rel))
                my_msg = String()
                my_msg.data = 'no'
                self.arming_status_pub.publish(my_msg)
                return('landed_crater')
            elif self.state_timer > 250: # timeout
                self.get_logger().error('Failed to land!')
                self.error = 'Failed_target_waypoint'
                return('follow_path_home')
            else:
                self.get_logger().info('Altitude: {}'.format(self.last_alt_rel))
                return('landing_on_crater')
        
        elif self.control_state == 'landed_crater':
            if self.interaction_command == 'return': 
                self.desired_altitude = 60.0
                my_msg = String()
                my_msg.data = 'yes'
                self.arming_status_pub.publish(my_msg)
                self.flyto(self.land_waypoints[self.waypoint_index_to_land][0], self.land_waypoints[self.waypoint_index_to_land][1], self.init_alt + self.desired_altitude - 47.0)
                return('climbing_crater') 
            elif self.interaction_command == 'abort': #command being sent by the user
                self.error = 'user_aborted'  #error to be sent to the interface engine
                return('follow_path_home')
            elif self.state_timer > 2500: # timeout 
                self.get_logger().error('Failed to takeoff') 
                self.error = 'timeout'
                return('follow_path_home') 
            else:
                self.get_logger().info('Landed on crater! Waiting for takeoff command!')
                return('landed_crater')

        elif self.control_state == 'climbing_crater':
            if self.last_alt_rel > 29.0:
                self.get_logger().info('Close enough to flight altitude') # move drone by sending setpoint message
                self.move_camera(60)
                return('follow_path_home')
            elif self.interaction_command == 'abort':
                self.error = 'user_aborted'
                return('follow_path_home')
            elif self.state_timer > 60: # timeout
                self.get_logger().error('Failed to reach altitude')
                self.error = 'Failed_target_height'
                return('landing')
            else:
                self.get_logger().info('Crater Climbing, altitude {}m'.format(self.last_alt_rel))
                return('climbing_crater')

        elif self.control_state == 'follow_path_home':
            if self.waypoint_index_to_land == -1:
                self.waypoint_index = self.stage_1_waypoints_latlong.shape[0]-1
                return('follow_path_home2')
            elif self.follow_path_home_initiated == False: 
                self.follow_path_home_initiated = True
                self.waypoint_index_to_land = self.waypoint_index_to_land-1
                return('follow_path_home')
            else:
                self.get_logger().info('Fly to:{},{}'.format(self.land_waypoints[self.waypoint_index_to_land][0],self.land_waypoints[self.waypoint_index_to_land][1]))
                self.flyto(self.land_waypoints[self.waypoint_index_to_land][0], self.land_waypoints[self.waypoint_index_to_land][1], self.init_alt + self.desired_altitude - 47.0) # unexplained correction factor on altitude
                return('on_way_home') 

        elif self.control_state == 'on_way_home':
            distance = self.distance_to_last()
            if abs(distance) < 0.00001:
                self.get_logger().info('Close enough to target distance={}'.format(distance))
                self.waypoint_index_to_land = self.waypoint_index_to_land-1
                return('follow_path_home')
            elif self.state_timer > 200: # timeout
                self.get_logger().error('Failed to reach target')
                self.error = 'Failed_target_waypoint'#James_edit
                return('landing')
            else:
                #self.get_logger().info('Target error {}'.format(distance))
                return('on_way_home')
           
        elif self.control_state == 'follow_path_home2':
            if self.waypoint_index == -1:
                return('landing')
            elif self.follow_path_home_initiated == False: 
                self.follow_path_home_initiated = True
                self.waypoint_index = self.waypoint_index-1
                return('follow_path_home2')
            else:
                self.get_logger().info('Fly to:{},{}'.format(self.stage_1_waypoints_latlong[self.waypoint_index][0],self.stage_1_waypoints_latlong[self.waypoint_index][1]))
                self.flyto(self.stage_1_waypoints_latlong[self.waypoint_index][0], self.stage_1_waypoints_latlong[self.waypoint_index][1], self.init_alt + self.desired_altitude - 47.0) # unexplained correction factor on altitude
                return('on_way_home2') 

        elif self.control_state == 'on_way_home2':
            distance = self.distance_to_last()
            if abs(distance) < 0.00001:
                self.get_logger().info('Close enough to target distance={}'.format(distance))
                self.waypoint_index = self.waypoint_index-1
                return('follow_path_home2')
            elif self.state_timer > 200: # timeout
                self.get_logger().error('Failed to reach target')
                self.error = 'Failed_target_waypoint'#James_edit
                return('landing')
            else:
                #self.get_logger().info('Target error {}'.format(distance))
                return('on_way_home2')
            
        elif self.control_state == 'landing': # return home and land
            self.change_mode("RTL")
            return('exit')

        elif self.control_state == 'exit': # nothing else to do
            my_msg = String()
            my_msg.data = 'no'
            self.arming_status_pub.publish(my_msg)
            return('exit')

    def timer_callback(self):
        new_state = self.state_transition()
        self.send_state()
        self.send_error()
        if new_state == self.control_state:
            self.state_timer = self.state_timer + 1
        else:
            self.state_timer = 0
        self.control_state = new_state
        self.get_logger().info('Controller state: {} for {} steps'.format(self.control_state, self.state_timer))
        
                    

def main(args=None):
    
    rclpy.init(args=args)

    controller_node = FenswoodDroneController()
    controller_node.start()
    rclpy.spin(controller_node)

if __name__ == '__main__':
    main()