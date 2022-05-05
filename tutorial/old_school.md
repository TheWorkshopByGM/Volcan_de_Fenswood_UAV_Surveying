# Old-School Example

[Back to tutorial contents](README.md#contents)

## Introduction

In this tutorial, a simple Python script will be used to control the drone, with the control flow managed just by the Python language flow.  This approached is referred to as 'old school' in the [minimal ROS2 examples](https://github.com/ros2/examples/tree/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber) and is _not_ good programming style.  However, it enables us to focus purely on the ROS interactions, stripped of any Python organization that may be unfamiliar to some readers.  Therefore, the old school way seems the right place to start.

> Style matters.  Good programming style makes code that is re-usable, maintainable, and likely to be correct.  There are some celebrated [lessons of style](https://en.wikipedia.org/wiki/The_Elements_of_Programming_Style#Lessons).  It is also a hotly debated topic in Python, which offers many ways of doing every task, but some more [Pythonic](https://www.udacity.com/blog/2020/09/what-is-pythonic-style.html) than others.

## Threads

ROS uses a [publish/subscribe](https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern) model to exchange data between components using named _topics_.  For example, for us to get the drone position, we learn [from the documentation](http://wiki.ros.org/mavros#mavros.2FPlugins.Published_Topics-2) that MAVROS will publish it to a topic named `/vehicle_1/mavros/global_position/global` using the `sensor_msgs/NavSatFix` message type.  Then we must subscribe to that topic to receive those messages.  When we subscribe, we write a _callback_ function that will run every time a position message is received and send its name when we subscribe.

This is a very powerful and scalable way of sharing information *but* it means our program is multi-threaded: different parts of it will run at different times, potentially on top of each other.  Python will handle _most_ of the pain, like worrying what if a message changes halfway through us processing it.  However, some ugliness remains: we will need to use global variables to allow the different threads to communicate with each other.  In our example this will be really simple: the callbacks will just save the messages as they come in, and our code will just look at the last one saved.

> You will need to think about this more when you start working with the camera feed.  You _could_ just put a load of image processing code in a callback, but what happens if you get a new image in while you're still processing the last one?

Our programme will have three threads:
 - the 'main' thread, executing the function `main`
 - the 'state' thread, executing the function `state_callback` whenever a state message is received
 - the `position` thread, executing the function `position_callback` whenever a positon message is received

## Example code

To run this example:
```
docker-compose -f docker-compose-old-school.yml up --build
```

The key file is [`fenswood_drone_controller/fenswood_drone_controller/controller_old_school.py`](https://github.com/StarlingUAS/fenswood_volcano_template/blob/main/fenswood_drone_controller/fenswood_drone_controller/controller_old_school.py) subdirectory.  The remainder of this section describes how it works.

```
import rclpy
```
The library `rclpy` contains the core functionality for using ROS from Python.
```
# import message definitions for receiving status and position
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
# import message definition for sending setpoint
from geographic_msgs.msg import GeoPoseStamped

# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong
```
ROS communications are all typed, so to use any ROS channels requires importing the relevant information type as a Python class.  There are two types of these definitions: _messages_ from `.msg` libraries and _services_ from `.srv` libraries: more on the distinction later.

Each class is imported from the ROS package that defines it.  For our example, we need:
 - the [State message from the `mavros_msgs` package](http://docs.ros.org/en/api/mavros_msgs/html/msg/State.html) which will convey the readiness status from the drone
 - the [NavSatFix message from the `sensor_msgs` package](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) which will tell us the position from the drone
 - the [GeoPoseStamped message from the `geographic_msgs` package](https://docs.ros.org/en/api/geographic_msgs/html/msg/GeoPoseStamped.html) which will take our position setpoint back to the drone
 - the services SetMode, CommandBool, CommandTOL, CommandLong from the `mavros_msg` package, which we will use to change modes, arm, take-off, and request data streams, respectively.

```
g_node = None           # global for the node handle
g_last_state = None     # global for last received status message
g_last_pos = None       # global for last received position message
g_init_alt = None       # global for global altitude at start
g_last_alt_rel = None   # global for last altitude relative to start
```
Recalling the earlier discussion about [threading](#threads) these global variables will be used to store information from the callback functions for use in the main control thread (or, in the case of the `g_node` variable, _vice versa_).  Initial values of `None` means we can trap cases when no data has been received.

> Global variables are horrible things.  You should really never use them as they can lead to all sorts of unexpected behaviour down the line - but they get us working quickly here.  They will be replaced in the nery next tutorial.

```
def state_callback(msg):
    global g_last_state
    g_last_state = msg
    g_node.get_logger().debug('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))
```
Above is our first callback function, which will run whenever we get a drone status message.  The `global` line is needed before we can write to the global variable of that name, not just a local copy.  Then all we do is save the incoming data in `msg` to the global and write a log entry.  There are different grades of logging and `debug()` is the lowest, meaning we won't get a screen clogged with these messages unless we go and ask for them.

> We didn't need a `global` declaration for g_node because we're only reading from it.  However, since the function writes to `g_last_state`, a `global g_last_state` is needed.  It's a Python thing.

```
def position_callback(msg):
    global g_last_pos, g_last_alt_rel
    # determine altitude relative to start
    if g_init_alt:
        g_last_alt_rel = msg.altitude - g_init_alt
    g_last_pos = msg
    g_node.get_logger().debug('Drone at {}N,{}E altitude {}m'.format(msg.latitude,
                                                                    msg.longitude,
                                                                    g_last_alt_rel))
```
Above is the other callback function, this time for the position message.  The `if g_init_alt:` clause will only run if `g_init_alt` has been set to something other than its initial value of `None`.  This means the `g_last_alt_rel` calculation only happens if `g_init_alt` has been set.  This is part of a workaround to work with altitude relative to take-off position rather than the ambiguous absolute value, as discussed in the [drone control tutorial](drone_control.md).  `g_init_alt` will be set later on when the drone is armed, after which the callback will also calculate the altitude relative to takeoff and store it in `g_last_alt_rel`.
```
def wait_for_new_status():
    """
    Wait for new state message to be received.  These are sent at
    1Hz so calling this is roughly equivalent to one second delay.
    """
    if g_last_state:
        # if had a message before, wait for higher timestamp
        last_stamp = g_last_state.header.stamp.sec
        for try_wait in range(60):
            rclpy.spin_once(g_node)
            if g_last_state.header.stamp.sec > last_stamp:
                break
    else:
        # if never had a message, just wait for first one          
        for try_wait in range(60):
            if g_last_state:
                break
            rclpy.spin_once(g_node)
```
Our simulation runs in real time and the controller will sometimes need to wait for things to happen.  Sadly we can't just use `time.sleep(1)` from the `time` Python library: this doesn't play nicely with ROS threads and ends up blocking all the callbacks as well.  Instead, we must use ROS `spin` methods to give up the time to the other threads.  Since we do this several times in the script, the function `wait_for_new_status` brings it all together.

The `if g_last_state:` line breaks it into two clauses.  If true, this means we have received a status message before.  The function grabs the seconds value of the timestamp of the last message in the variable `last_stamp`.  Then the `for` loop runs until a new message with a later timestamp has been received.  The `spin_once()` command waits until one of the other threads has done some work.  If we had never received a message before, the first `if g_last_state:` fails, and instead we just keep spinning until any message has been received.

In testing, the state messages come in at 1Hz, so this function waits for about 1s.  Be careful relying on that though: if a ground station were to request state messages at a different rate, that delay time could change.

> I could have done this with a `while` loop, but the `for` loop is better practice for real world control, as it means the program cannot hang forever and will eventually time out.  However, I have failed to handle this timeout (or any of the others in the example) properly: the programme will just carry on.  Python provides a lovely `else` syntax (see [documentation](https://docs.python.org/3/tutorial/controlflow.html#break-and-continue-statements-and-else-clauses-on-loops)) that would handle this very elegantly.

```
def main(args=None):
    global g_node, g_init_alt
```
The `main` function will carry the main thread of our program, including all the decision-making in this example.  The `global` line means we will be writing to a couple of variables, as discussed already.  
```
    rclpy.init(args=args)
    g_node = rclpy.create_node('example_controller')
```
The above are two bits of ROS 'magic'.  Every process that talks over ROS is called a 'node' and has to register itself with the rest of the ROS environment and give itself a name.  You will see this name turn up in the logs on Foxglove to identify which message comes from which node.  
```
    state_sub = g_node.create_subscription(State, '/vehicle_1/mavros/state', state_callback, 10)
    pos_sub = g_node.create_subscription(NavSatFix, '/vehicle_1/mavros/global_position/global', position_callback, 10)
```
This sets up our subscribers, specifying the message types, topic names, and callback functions for each.

> Don't worry about the `10` for now.

These lines create the two additional threads besides the main one, and once they've been executed, callbacks can start running.

Now the script just works through the [steps described in the Drone control tutorial](drone_control.md#example-steps)
```
    for try_standby in range(60):
        wait_for_new_status()
        if g_last_state.system_status==3:
            g_node.get_logger().info('Drone ready for flight')
            break 
```
First, above, we wait up to a minute (assuming `wait_for_new_status` takes one second) to reach the 'standby' status (3).

> If we reach one minute and standby hasn't been reached, the code just moves on anyway.
```
    # send command to request regular position updates
    cmd_req = CommandLong.Request()
    cmd_req.command = 511
    cmd_req.param1 = float(33)  # msg ID for position is 33 \
                                # https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
    cmd_req.param2 = float(1000000)    # 1000000 micro-second interval : 1Hz rate
```
The next step is to request the position data.  MAVROS doesn't give us a direct route to do this, but it does provide a _ROS service_ for us to send any MAVLINK message of our choice.  The code above compiles a service request of type `mavros_msgs/CommandLong` to perform the request.  Following [these instructions](https://ardupilot.org/dev/docs/mavlink-requesting-data.html), we send a MAVLINK [SET_MESSAGE_INTERVAL message, number 511](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL) using `param1` to identify the message we want, [number 33, GLOBAL_POSITION_INT](https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT) and `param2` to set the interval in microseconds.  MAVROS will get upset if the `param` values are not `float` variables, hence the conversions.
```
    cmd_cli = g_node.create_client(CommandLong, '/vehicle_1/mavros/cmd/command')
    while not cmd_cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('command_int service not available, waiting again...')
    future = cmd_cli.call_async(cmd_req)
    rclpy.spin_until_future_complete(g_node, future)    # wait for response
    g_node.get_logger().info('Requested position stream')
```
*Services* are [another way of communicating in ROS](https://docs.ros.org/en/foxy/Tutorials/Services/Understanding-ROS2-Services.html), an alternative to topic publishing and subscribing.  Services are to topics what phone calls are to text messages.  A service is _called_ by a client and a server then _responds_.

In our case, we learn [from the documentation](http://wiki.ros.org/mavros#mavros.2FPlugins.Services) that MAVROS provides the function we need as a service named `\vehicle_1\mavros\cmd\command` with defined message type `mavros_msgs/CommandLong`.  Having constructed our `CommandLong` in the previous code chunk, we start here by creating a 'client' for calling the service named `cmd_cli` and then wait to make sure the corresponding server is ready using `while not cmd_cli.wait_for_service()`.

As services have the risk of delaying or even deadlocking your code, ROS gives some rather complicated mechanisms for calling them.  The `call_async()` command starts (yet) another thread to wait for the response, and all I do is use `spin_until_future_complete()` to just wait for the response and then log that it finished.  The response data would be in the `future` object after the `spin` is done.  Lazily, I just ignore it, assume it worked, and move on.

> Generally, I am not a fan of ROS services.  ROS2 has had to make them [rather complicated](https://docs.ros.org/en/foxy/How-To-Guides/Sync-Vs-Async.html) to overcome the problems of deadlocks, when nodes end up waiting for each other to respond to services.  I can see the advantage of having explicit responses to some messages, but the associated complexity is a pain.  The old [Parrot ARDrone ROS drivers](http://wiki.ros.org/ardrone_autonomy) managed to do absolutely everything with just a few topics and no services.  Meanwhile, the newer [ROS actions](https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Actions.html) provide a much better interface for call-and-response behaviour.  Lots of really useful tools like [py_trees_ros](https://github.com/splintered-reality/py_trees_ros) support topics and actions but not services.  Still, we are lucky that others have produced MAVROS for us, so it's worth a little pain to accommodate their choices.  

```
    mode_req = SetMode.Request()
    mode_req.custom_mode = "GUIDED"
    mode_cli = g_node.create_client(SetMode, '/vehicle_1/mavros/set_mode')
    while not mode_cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('set_mode service not available, waiting again...')
    future = mode_cli.call_async(mode_req)
    rclpy.spin_until_future_complete(g_node, future)    # wait for response
    g_node.get_logger().info('Request sent for GUIDED mode.')
```
Changing mode requires another service call.  Hopefully the pattern is emerging: build a service request `mode_req`, then a client `mode_cli`, wait for the service, call it, `spin_until_future_complete` to wait for completion, and then move on.  None of the service calls in this tutorial ever check the response.  If necessary, I you could verify the mode change by looking at the state message. 
```    
    arm_req = CommandBool.Request()
    arm_req.value = True
    arm_cli = g_node.create_client(CommandBool, '/vehicle_1/mavros/cmd/arming')
    while not arm_cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('arming service not available, waiting again...')
    # keep trying until arming detected in state message, or 60 attempts
    for try_arm in range(60):
        future = arm_cli.call_async(arm_req)
        rclpy.spin_until_future_complete(g_node, future)
        g_node.get_logger().info('Arming request sent.')
        wait_for_new_status()
        if g_last_state.armed:
            g_node.get_logger().info('Arming successful')
            # armed - grab init alt for relative working
            if g_last_pos:
                g_init_alt = g_last_pos.altitude
            break
    else:
        g_node.get_logger().error('Failed to arm')
```
Arming the drone is done by calling yet another service.  Again it begins by constructing the `arm_req` request and the `arm_cli` client.  However, since we anticipate arming to fail while the GPS _etc_ is still warming up, the code here repeats for up to 60 attempts, calling the service using `call_async`, doing a `spin_until_future_complete` until the service completes, waiting for a new status message, and only stopping via `break` if that status shows `armed` to be `True`.  If arming succeeds, the code also grabs the current altitude in `g_init_alt` to enable relative altitude calculation.

The `else:` clause will only run if the `for` loop makes it to its full 60 interations without terminating via a break.  It therefore handles the case where arming times out, sending an `error` message to the log.
```
    takeoff_req = CommandTOL.Request()
    takeoff_req.altitude = 20.0
    takeoff_cli = g_node.create_client(CommandTOL, '/vehicle_1/mavros/cmd/takeoff')
    while not takeoff_cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('takeoff service not available, waiting again...')
    future = takeoff_cli.call_async(takeoff_req)
    rclpy.spin_until_future_complete(g_node, future)
    g_node.get_logger().info('Takeoff request sent.')
```
Take-off is achieved by yet another service call following a hopefully familiar pattern.  This time the request includes the `altitude` to which the drone should climb, which in this circumstance is always interpreted relative to ground level.
```
    # wait for drone to reach desired altitude, or 600 attempts
    for try_alt in range(600):
        wait_for_new_status()
        g_node.get_logger().info('Climbing, altitude {}m'.format(g_last_alt_rel))
        if g_last_alt_rel > 19.0:
            g_node.get_logger().info('Close enough to flight altitude')
            break
```
The above snippet waits for 600 seconds or for the drone to reach 19m above its arming altitude.  Note the `g_last_alt_rel` variable will be calculated in the `state_callback` function (in its own thread) as the `g_init_alt` variable has been set in the main thread.
```
    # move drone by sending setpoint message
    target_msg = GeoPoseStamped()
    target_msg.pose.position.latitude = 51.423
    target_msg.pose.position.longitude = -2.671
    target_msg.pose.position.altitude = g_init_alt + 20.0 - 50.0 # MSL/ellipsoid correction
```
Time to get the drone moving.  Start by composing a `GeoPoseStamped()` message with a target location.  The correction factor `-50.0` accounts for the differences between different altitude definitions - it's something of a hack but seems to work OK for this short move.
```
    target_pub = g_node.create_publisher(GeoPoseStamped, '/vehicle_1/mavros/setpoint_position/global', 10)
    wait_for_new_status() # short delay after creating publisher ensures message not lost
    target_pub.publish(target_msg)
    g_node.get_logger().info('Sent drone to {}N, {}E, altitude {}m'.format(target_msg.pose.position.latitude,
                                                                           target_msg.pose.position.longitude,
                                                                           target_msg.pose.position.altitude)) 
```
And publishing is as simple as this - create a publisher object and then call its `publish` method to send the message.  *Note* the `wait_for_new_status()` call between creating the publisher and using it - I've found that using publishers seem to take a little time to get ready, and if you try and publish straight after creating, the message often vanishes.
```
    # wait for drone to reach desired position, or timeout after 600 attempts
    for try_arrive in range(600):
        wait_for_new_status()
        d_lon = g_last_pos.longitude - target_msg.pose.position.longitude
        d_lat = g_last_pos.latitude - target_msg.pose.position.latitude
        g_node.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
        if abs(d_lon) < 0.0001:
            if abs(d_lat) < 0.0001:
                g_node.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                break
```
The drone should now be moving.  The position callbacks will be running in their own thread, so I can just access the last received message in the `g_last_pos` global.  When both numbers match to four decimal places (or on timeout after 600 seconds) I declare the target reached and allow the code to move on.
```
    mode_req.custom_mode = "RTL"
    future = mode_cli.call_async(mode_req)
    rclpy.spin_until_future_complete(g_node, future)    # wait for response
    g_node.get_logger().info('Request sent for RTL mode.')
```
The final step is to send the drone back home.  The mode request `mode_req` and mode client `mode_cli` are already set up from earlier, and the `set_mode` service has already been proven available with an earlier `wait_for_service()` so there's no need for another.  I just change the desired mode in `mode_req`, call the service again and do another `spin_until_future_complete` to let it finish.
```
    # now just serve out the time until process killed
    while rclpy.ok():
        rclpy.spin_once(g_node)
```
The control task is complete now.  I _ought_ to monitor the drone's progress until it lands and exit the program cleanly.   Lazily, I just keep executing `spin_once()` to keep the callbacks running until `rclpy.ok()` fails, which happens when the application is killed with a `Ctrl-C`.
```
if __name__ == '__main__':
    main()
```
This last bit above is another 'Python thing'.  It detects the special case where the script is executed as a program (instead of just imported as a module) and directs execution to the `main()` function.  I'm not even sure it's needed with ROS2 but it's habit.

## Exercises

1. Change where the drone flies, perhaps trying some of the locations provided in the project briefing document.

2. Add a second location, so the drone flies to the first location and then on to a second.

3. Add a request for [message 32, LOCAL_POSITION_NED](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED) to be sent.  Observe what happens on foxglove looking at topic `/vehicle_1/mavros/local_position/pose`.  In the code, add a subscriber to `/vehicle_1/mavros/local_position/pose` and print the local information to the log.

5. Instead of sending the drone back to the starting location, trying using `Land` mode to land it at the target location.

6. Add a final stage to the program that checks the drone makes it back to the ground, exits gracefully if it does, or logs an error if it takes too long.