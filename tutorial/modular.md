# A more modular controller

[Back to tutorial contents](README.md#contents)

## Introduction

In the previous [simple class](simple_class.md) example, a controller class was created, of which the callbacks and the main control thread were methods, and the items of shared data were properties.  The class was a child of the ROS `Node` class meaning that all the ROS functionality was inherited.  This made a rather elegant structure - but was only the beginning.  In this example, the main steps will be broken out into additional methods of the `FenswoodDroneController` class, so the main `run` method can be simpler.  The result is a more _modular_ and extensible solution.  ["Each module should do one thing well."](https://reprog.wordpress.com/2010/03/06/programming-books-part-2-the-elements-of-programming-style/)  Achieve that, and you can start re-using those modules to do more complex things, also well.

## Example code

To run this example:
```
docker-compose -f docker-compose-modular.yml up --build
```

The key file is [`fenswood_drone_controller/fenswood_drone_controller/controller_modular.py`](../fenswood_drone_controller/fenswood_drone_controller/controller_modular.py).  Only differences from the [first object-oriented solution](simple_class.md#example-code) are highlighted.

```
import rclpy
from rclpy.node import Node

# import message definitions for receiving status and position
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
# import message definition for sending setpoint
from geographic_msgs.msg import GeoPoseStamped

# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong


class FenswoodDroneController(Node):

    def __init__(self):
        super().__init__('example_controller')
        self.last_state = None     # global for last received status message
        self.last_pos = None       # global for last received position message
        self.init_alt = None       # global for global altitude at start
        self.last_alt_rel = None   # global for last altitude relative to start
```
The code starts off with the same imports and initialization as the [simple class implementation](simple_class.md#example-code). 
```
        # create service clients for long command (datastream requests)...
        self.cmd_cli = self.create_client(CommandLong, 'mavros/cmd/command')
        while not self.cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('command_int service not available, waiting again...')
        # ... for mode changes ...
        self.mode_cli = self.create_client(SetMode, 'mavros/set_mode')
        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')
        # ... for arming ...
        self.arm_cli = self.create_client(CommandBool, 'mavros/cmd/arming')
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')
        # ... and for takeoff
        self.takeoff_cli = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        while not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')
        # create publisher for setpoint
        self.target_pub = self.create_publisher(GeoPoseStamped, 'mavros/setpoint_position/global', 10)
        # and make a placeholder for the last sent target
        self.last_target = GeoPoseStamped()
```
Now, all the clients for services and publishing are created in the constructor `__init__()`.  This is so they can be used and re-used by methods coming in a bit.  Note the client handles (_e.g._ `self.takeoff_cli`) are stored as properties of the object, using `self`.  This means thay can be accessed by other methods of the same object, again via `self`.

Only publishers and service callers are included here.  Subscribers are still left for the `run()` method as I don't want stuff to start running until later on.

You _could_ have kept these activities in other methods, creating new clients each time something needs to be published or called.  However, they take work and delay due to interactions with the ROS ecosystem, so it's better to just get them done once at the start.

> Arguably, I should have put the `wait_for_service` calls in `run()` as well, so an object could be constructed in isolation from ROS if I wanted to.

```
        
    # on receiving status message, save it to global
    def state_callback(self,msg):
        self.last_state = msg
        self.get_logger().debug('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))

    # on receiving positon message, save it to global
    def position_callback(self,msg):
        # determine altitude relative to start
        if self.init_alt:
            self.last_alt_rel = msg.altitude - self.init_alt
        self.last_pos = msg
        self.get_logger().debug('Drone at {}N,{}E altitude {}m'.format(msg.latitude,
                                                                        msg.longitude,
                                                                        self.last_alt_rel))

    def wait_for_new_status(self):
        """
        Wait for new state message to be received.  These are sent at
        1Hz so calling this is roughly equivalent to one second delay.
        """
        if self.last_state:
            # if had a message before, wait for higher timestamp
            last_stamp = self.last_state.header.stamp.sec
            for try_wait in range(60):
                rclpy.spin_once(self)
                if self.last_state.header.stamp.sec > last_stamp:
                    break
        else:
            # if never had a message, just wait for first one          
            for try_wait in range(60):
                if self.last_state:
                    break
                rclpy.spin_once(self)

```
Callbacks and the `wait_for_status()` utility method are unchanged: these are already nice simple functions with clear, distinct purposes.
```
    def request_data_stream(self,msg_id,msg_interval):
        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(msg_id)
        cmd_req.param2 = float(msg_interval)
        future = self.cmd_cli.call_async(cmd_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response
```
Above is the first of the new methods to take on a specific task: in the case, requesting a data stream.  "Make sure every module hides something" is [another good guideline](https://www.parkerklein.com/notes/the-elements-of-programming-style).  Here, we hide from later software that requesting data involves all this complicated stuff about long commands and service calls.  The function takes the desired message `msg_id` and the message interval `msg_interval` as arguments so future calls can request different messages and rates.  With the relevant service client already set up in the `__init__` constructor, we can go straight ahead and call `self.cmd_cli.call_async()` without the set-up and waiting. 
```
    def change_mode(self,new_mode):
        mode_req = SetMode.Request()
        mode_req.custom_mode = new_mode
        future = self.mode_cli.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response
```
Another helper method, `change_mode` does exactly what it says in the name, again hiding the associated ROS details under a simple Python call.  For re-use, callers provide the `new_mode` as an argument.
```
    def arm_request(self):
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arm_cli.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future)
```
Hopefully the pattern is emerging: each of the steps of our control flow are now going to be single lines, calling the associated methods and hiding the ROS interfaces.  `arm_request` doesn't need any arguments as it always does the same job.
```
    def takeoff(self,target_alt):
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = target_alt
        future = self.takeoff_cli.call_async(takeoff_req)
        rclpy.spin_until_future_complete(self, future)

    def flyto(self,lat,lon,alt):
        self.last_target.pose.position.latitude = lat
        self.last_target.pose.position.longitude = lon
        self.last_target.pose.position.altitude = alt
        self.target_pub.publish(self.last_target)
        self.get_logger().info('Sent drone to {}N, {}E, altitude {}m'.format(lat,lon,alt)) 

```
Methods for `take_off` and `flyto` are hopefully self-explanatory, and show how Python code can just about comment itself if you choose the names clearly.  Note that `flyto()` saves the commanded target to a property `last_target` for later use.

> I was trying to follow the [Python style guide on function names](https://www.python.org/dev/peps/pep-0008/#function-and-variable-names) so `flyto` really ought to be `fly_to`.  I recommend you adopt a standard and stick to it.  For all it can feel pedantic sometimes, it really does help when working on code as a group.

```
    def run(self):
        # set up two subscribers, one for vehicle state...
        state_sub = self.create_subscription(State, 'mavros/state', self.state_callback, 10)
        # ...and the other for global position
        pos_sub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.position_callback, 10)

        # first wait for the system status to become 3 "standby"
        # see https://mavlink.io/en/messages/common.html#MAV_STATE
        for try_standby in range(60):
            self.wait_for_new_status()
            if self.last_state.system_status==3:
                self.get_logger().info('Drone ready for flight')
                break 
```
The `run()` method starts off just as always, launching the subscriptions and waiting for the right status.
```
        # send command to request regular position updates
        self.request_data_stream(33, 1000000)
        self.get_logger().info('Requested position stream')

        # now change mode to GUIDED
        self.change_mode("GUIDED")
        self.get_logger().info('Request sent for GUIDED mode.')
```
Now the next two steps, asking for the position data and changing to GUIDED, are just two lines of code plus associated logging.  It's much easier to follow the intent than before with the complexity of all the ROS moved to the methods.

> Why did I not put the `get_logger().info()` calls in the methods too?  Don't know really - I did for `flyto` but not for the other methods.  That inconsistency irks me now, but it made sense at the time.

```        
        # next, try to arm the drone
        # keep trying until arming detected in state message, or 60 attempts
        for try_arm in range(60):
            self.arm_request()
            self.get_logger().info('Arming request sent.')
            self.wait_for_new_status()
            if self.last_state.armed:
                self.get_logger().info('Arming successful')
                # armed - grab init alt for relative working
                if self.last_pos:
                    self.init_alt = self.last_pos.altitude
                break
        else:
            self.get_logger().error('Failed to arm')
```
The arming step is slightly simpler with just the `arm_request` call doing the work.
```
        # take off and climb to 20.0m at current location
        self.takeoff(20.0)
        self.get_logger().info('Takeoff request sent.')
```
Take off is now just one line calling the earlier method, plus a log record. 
```
        # wait for drone to reach desired altitude, or 60 attempts
        for try_alt in range(60):
            self.wait_for_new_status()
            self.get_logger().info('Climbing, altitude {}m'.format(self.last_alt_rel))
            if self.last_alt_rel > 19.0:
                self.get_logger().info('Close enough to flight altitude')
                break

        # move drone by sending setpoint message
        self.flyto(51.423, -2.671, self.init_alt - 30.0) # unexplained correction factor on altitude
```
Moving the drone is again just a single line.
```
        # wait for drone to reach desired position, or timeout after 60 attempts
        for try_arrive in range(60):
            self.wait_for_new_status()
            d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
            d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
            self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
            if abs(d_lon) < 0.0001:
                if abs(d_lat) < 0.0001:
                    self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                    break
```
Monitoring progress is largely unchanged, apart from using the stored `last_target` property to check distance.  This actually feels clumsy compared to the rest of the code and I will ask you to improve upon it in the exercises.
```
        # return home and land
        self.change_mode("RTL")
        self.get_logger().info('Request sent for RTL mode.')
```
The `change_mode` method is used again to head home using RTL.
```     
        # now just serve out the time until process killed
        while rclpy.ok():
            rclpy.spin_once(self)


def main(args=None):
    
    rclpy.init(args=args)

    controller_node = FenswoodDroneController()
    controller_node.run()


if __name__ == '__main__':
    main()
```
The remainder is unchanged.

[Back to tutorial contents](README.md#contents)

## Exercises

All exercises involve editing the file [`fenswood_drone_controller/fenswood_drone_controller/controller_modular.py`](../fenswood_drone_controller/fenswood_drone_controller/controller_modular.py)

1. Create a method to move the camera.  It should publish a [message of type `std_msgs/Float32`](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html) to the `/vehicle_1/gimbal_tilt_cmd` topic to move the camera, with `0` in the `data` field being horizontal and `1.57` being straight downwards.

2. Add a method to measure distance (in some simple way) to the last target.  Use it to simplify the code in the `run()` method.  Then add a stage to your code that flies to a second target after the first has been reached.

> None of the methods in the example so far returned a value!  If you include a `return x` statement in the method, you can use `x = self.my_method(stuff)` to access it.  Use `return a,b,c` and `a,b,c = self.my_method(stuff)` to return multiple values , if that's required.

3. Add a request for [message 32, LOCAL_POSITION_NED](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED).  Observe what happens on foxglove looking at topic `/vehicle_1/mavros/local_position/pose`.  In the code, add a subscriber to `/vehicle_1/mavros/local_position/pose` and try using this local altitude to track climb progress in `run()`.

[Back to tutorial contents](README.md#contents)