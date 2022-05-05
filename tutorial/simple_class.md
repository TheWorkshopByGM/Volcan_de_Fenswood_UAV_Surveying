# Implementing the controller as a class

[Back to tutorial contents](README.md#contents)

## Introduction

We can improve on the ugliness of a monolithic script and global variables using Object-Oriented Programming (OOP).  A full introduction to OOP is beyond the scope of this tutorial, so it is expected that you are familiar with the fundamentals: classes, objects, methods, properties, methods and inheritance.  By making our controller an _object_, it is able to share data between its different _methods_ (in the callbacks and the main thread) without using ugly global variables.  Furthermore, we can make our controller a child of the ROS `Node` class which tidies up a lot of ROS interaction, inspired by the [minimal ROS examples](https://github.com/ros2/examples/blob/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py). 

[Back to tutorial contents](README.md#contents)

## Example code

To run this example:
```
docker-compose -f docker-compose-simple-class.yml up --build
```

The key file is [`fenswood_drone_controller/fenswood_drone_controller/controller_simple_class.py`](../fenswood_drone_controller/fenswood_drone_controller/controller_simple_class.py) subdirectory.  The remainder of this section describes how it works.  Only differences from the [old school](old_school.md#example-code) are covered and details may refer back to explanations in that example.

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
```

The initial imports are the same as [before](old_school.md#example-code), with the exception of the extra import of the `Node` class from `rclpy` which we will use as the parent of our controller.

```
class FenswoodDroneController(Node):
```
Straight into the definition of our custom `FenswoodDroneController` class which will be a child of the ROS `Node` class.  This means it will have all the same methods and properties as a `Node`, plus any extra we add and any changes we make by overloading.

```
    def __init__(self):
        super().__init__('example_controller')
```
The `__init__` function (the constructor) of the parent `Node` class is called explicitly, setting any `FenswoodDroneController` up as a ROS `Node` with the name provided.
```
        self.last_state = None     # global for last received status message
        self.last_pos = None       # global for last received position message
        self.init_alt = None       # global for global altitude at start
        self.last_alt_rel = None   # global for last altitude relative to start
```
Stores for information shared between different parts of the class are initialized here as properties of the controller, referenced by the `self` keywords.  These relate one-to-one with the [old global variables](old_school.md#example-code) but will be local to the controller object.  Thus if we ever made more than one controller node, each would have its own data: much better!
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
```
The callbacks are defined as methods of the `FenswoodDroneController` class, as they are indented to be part of the `class` definition.  They therefore have access to the properties via the `self` command and we don't need any nasty `global` stuff.  The functionalisyt is the same as before: just save the messages as they come in, and calculate a relative altitude if an initial altitude has been set.
```
    def wait_for_new_status(self):
        if self.last_state:
            # if had a message before, wait for higher timestamp
            last_stamp = self.last_state.header.stamp.sec
            for try_wait in range(60):
                rclpy.spin_once(self)
                if self.last_state.header.stamp.sec > last_stamp:
                    break
        else:
            for try_wait in range(60):
                if self.last_state:
                    break
                rclpy.spin_once(self)
```
The `wait_for_new_status` function is unchanged, save for becoming a method.  As before, if a previous status message has been received, wait for a newer one, otherwise wait for the first one.
```
    def run(self):
        # set up two subscribers, one for vehicle state...
        state_sub = self.create_subscription(State, 'mavros/state', self.state_callback, 10)
        # ...and the other for global position
        pos_sub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.position_callback, 10)
```
The `run` method executes all the control activities (like the former `main` function).  It starts by subscribing to the state and global position topics.  _Note:_ you could put the subscribers in the `__init__` function, and the functionality of the code would be identical in this example.  However, I always prefer to put subscribers in a separate `run` or `start` method.  That way, you can instantiate an object from the controller class without firing up all the callbacks, which is helpful if you sometimes want to do other tasks separate to the control work.

Because our class is a child of the `Node` class, methods of that parent class can be accessed via `self`, such as `self.create_subscription`.

```
        # first wait for the system status to become 3 "standby"
        # see https://mavlink.io/en/messages/common.html#MAV_STATE
        for try_standby in range(60):
            self.wait_for_new_status()
            if self.last_state.system_status==3:
                self.get_logger().info('Drone ready for flight')
                break 

        # send command to request regular position updates
        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(33)  # msg ID for position is 33 \
                                    # https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
        cmd_req.param2 = float(1000000)    # 1000000 micro-second interval : 1Hz rate
        cmd_cli = self.create_client(CommandLong, 'mavros/cmd/command')
        while not cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('command_int service not available, waiting again...')
        future = cmd_cli.call_async(cmd_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response
        self.get_logger().info('Requested position stream')

        # now change mode to GUIDED
        mode_req = SetMode.Request()
        mode_req.custom_mode = "GUIDED"
        mode_cli = self.create_client(SetMode, 'mavros/set_mode')
        while not mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')
        future = mode_cli.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response
        self.get_logger().info('Request sent for GUIDED mode.')
        
        # next, try to arm the drone
        arm_req = CommandBool.Request()
        arm_req.value = True
        arm_cli = self.create_client(CommandBool, 'mavros/cmd/arming')
        while not arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')
        # keep trying until arming detected in state message, or 60 attempts
        for try_arm in range(60):
            future = arm_cli.call_async(arm_req)
            rclpy.spin_until_future_complete(self, future)
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

        # take off and climb to 20.0m at current location
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = 20.0
        takeoff_cli = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        while not takeoff_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')
        # only call once - seems to work OK
        future = takeoff_cli.call_async(takeoff_req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Takeoff request sent.')

        # wait for drone to reach desired altitude, or 60 attempts
        for try_alt in range(60):
            self.wait_for_new_status()
            self.get_logger().info('Climbing, altitude {}m'.format(self.last_alt_rel))
            if self.last_alt_rel > 19.0:
                self.get_logger().info('Close enough to flight altitude')
                break

        # move drone by sending setpoint message
        target_msg = GeoPoseStamped()
        target_msg.pose.position.latitude = 51.423
        target_msg.pose.position.longitude = -2.671
        target_msg.pose.position.altitude = self.init_alt - 30.0 # unexplained correction factor
        target_pub = self.create_publisher(GeoPoseStamped, 'mavros/setpoint_position/global', 10)
        self.wait_for_new_status() # short delay after creating publisher ensures message not lost
        target_pub.publish(target_msg)
        self.get_logger().info('Sent drone to {}N, {}E, altitude {}m'.format(target_msg.pose.position.latitude,
                                                                            target_msg.pose.position.longitude,
                                                                            target_msg.pose.position.altitude)) 

        # wait for drone to reach desired position, or timeout after 60 attempts
        for try_arrive in range(60):
            self.wait_for_new_status()
            d_lon = self.last_pos.longitude - target_msg.pose.position.longitude
            d_lat = self.last_pos.latitude - target_msg.pose.position.latitude
            self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
            if abs(d_lon) < 0.0001:
                if abs(d_lat) < 0.0001:
                    self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                    break

        # return home and land
        mode_req.custom_mode = "RTL"
        future = mode_cli.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response
        self.get_logger().info('Request sent for RTL mode.')
        
        # now just serve out the time until process killed
        while rclpy.ok():
            rclpy.spin_once(self)
```

The rest of this method is almost identical to the [old school example](old_school.md#example-code) but with the globals replaced by the properties.  Note also extensive use of the `Node` methods, _e.g._ `self.get_logger()` or `self.create_publisher()`.

```
def main(args=None):
    
    rclpy.init(args=args)

    controller_node = FenswoodDroneController()
    controller_node.run()

```
We still have to have a `main` function as the entry point for the script, but now it just creates a new object of the `FenswoodDroneController` class and then executes its `run()` method.
```

if __name__ == '__main__':
    main()
```
The above is the same 'Python thing' as before, redirecting to the `main` function if this file is executed as a script.

## Exercises


All exercises involve editing the file [`fenswood_drone_controller/fenswood_drone_controller/controller_simple_class.py`](../fenswood_drone_controller/fenswood_drone_controller/controller_simple_class.py)

1. Move the camera.  Publish a [message of type `std_msgs/Float32`](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html) to the `/vehicle_1/gimbal_tilt_cmd` topic to move the camera, with `0` in the `data` field being horizontal and `1.57` being straight downwards.

[Back to tutorial contents](README.md#contents)
