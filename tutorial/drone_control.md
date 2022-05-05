# Controlling an Ardupilot drone using MAVLINK over ROS

[Back to tutorial contents](README.md#contents)

## Introduction

The application includes a representation of the [Ardupilot copter](https://ardupilot.org/copter/docs/introduction.html) autopilot, a free open-source product in common use in aerial robotics both commercially and in research.  Instead of running on a physical autopilot like the [Pixhawk](https://pixhawk.org/), the application uses its [simulation capability](https://ardupilot.org/copter/docs/common-simulation.html) to run it on your computer.  Hence you must learn the 'fly the autopilot'.  

> While the standard interface for the drone is [MAVLINK](https://mavlink.io/en/), you will interact using ROS, with the conversion handled by the [MAVROS package](http://wiki.ros.org/mavros).

## Example steps

These are the steps performed by the example code.  All versions use these same steps.  To follow along, I recommend running the `old_school` version using 
```
docker-compose -f docker-compose-old-school.yml up --build
``` 
and then using <a target="_blank" href="https://studio.foxglove.dev">foxglove</a> (see [Introspecting ROS](../README.md#introspecting-ros)).  You should see reports on the ROS logs about progress through the steps.

 1. Wait for the autopilot to initialize, indicated by a `system_status` value of 3, [`MAV_STATE_STANDBY`](https://mavlink.io/en/messages/common.html#MAV_STATE_STANDBY), on the [`/vehicle_1/mavros/state`](http://wiki.ros.org/mavros#mavros.2FPlugins.sys_status) ROS topic.

 2. Request regular updates on global (GPS) position.  While most off-the-shelf control software takes care of this, our code needs to explicitly [request this information](https://ardupilot.org/dev/docs/mavlink-requesting-data.html) from the autopilot.  It is done by sending a custom MAVLINK message using the [`/vehicle_1/mavros/cmd/command`](http://wiki.ros.org/mavros#mavros.2FPlugins.command) ROS service.

    > There is also a local position available, in Cartesian coordinates relative to the home position.  This has some advantages, especially for altitude, but would take more work given we need to work with latitude and longitude.

  3. Change flight mode to `Guided`, meaning that the drone will track a position command from our software.  Ardupilot provides lots of different [flight modes](https://ardupilot.org/copter/docs/flight-modes.html) such as:
    - `Stabilise` pilot flies using joysticks, everything else ignored
    - `Loiter` drone uses GPS to hold position, with movements commanded by pilot using joysticks
    - `Land` drone lands at current location
    - `RTL` drone goes back to home position and lands
    - `Auto` drone follows a set of uploaded mission commands 

    > Although you can change mode using software, try not to do it too often.  One of the safety procedures is for the pilot to take control by changing to `Stabilise` or `Loiter` mode, and that becomes harder if your software keeps trying to change back.

  4. [Arm the drone](https://ardupilot.org/copter/docs/arming_the_motors.html).  This stage starts the motors spinning, hence you only want to do it right before take-off.  It can be done by software or by the pilot using joysticks.  Arming involves certain checks of things like GPS stability and position estimation, which sometimes take about a minute to settle after powering up.  Hence you can expect to need a few goes before the drone successfuly arms.  

    > Think about safety when deciding how to arm.  You must not start the motors spinning unless you're sure there is no-one working on the drone, _e.g._ completing the battery fitting.  In the simulated examples, the software does it as soon as the drone is ready.  In practice, you might want pilot confirmation, or just leave it to the pilot themselves.

  5. Take-off!  This is a simple software command including a specified target altitude.

    > If you don't take off within a few seconds of arming, the drone will automatically disarm, for safety reasons.

  6. Climb, waiting for the drone to get close to the target altitude.  Use the information on the `/vehicle_1/mavros/global_position` [topics](http://wiki.ros.org/mavros#mavros.2FPlugins.global_position) to monitor progress.

    > The drone will accept a new setpoint at any altitude.  However, for the safety of those likely to be near the takeoff point, it is undesirable for the drone to do anything other than climb straight up at first.

    > Altitude is a surprisingly complex topic.  It can be expressed relative to mean sea level, to a standardized Earth shape model, to the location where you armed the drone, or to the local atmosphere using air pressure.  These can be many tens of metres different to each other.  [Solutions are available](http://wiki.ros.org/mavros#mavros.2FPlugins.Avoiding_Pitfalls_Related_to_Ellipsoid_Height_and_Height_Above_Mean_Sea_Level) but I find them rather complicated.  Instead I recommend pragmatic solutions like using relative information and calculated correction factors, as we are not flying far.  *Note* there is also a helpful-looking `/vehicle_1/mavros/global_position/rel_alt` topic that I have not tried yet.

  7. Move the drone by sending a target, in terms of latitude, longitude and altitude, over the `/vehicle_1/mavros/setpoint_position/global` [topic](http://wiki.ros.org/mavros#mavros.2FPlugins.setpoint_position).  *Note* the altitude issues described above really bite here.  The position input uses a different altitude convention to the GPS output.

    > You can also command the drone to a fixed velocity instead of a position, using the `/vehicle_1/mavros/setpoint_velocity/cmd_vel_unstamped` [topic](http://wiki.ros.org/mavros#mavros.2FPlugins.setpoint_velocity).  Helpfully, this is a ROS `Twist` message which is a standard for movement commands.

  8. Wait for the drone to reach its target, or get close enough, by monitoring the `/vehicle_1/mavros/global_position` [topics](http://wiki.ros.org/mavros#mavros.2FPlugins.global_position) again.

    > Calculating distance between two latitude and longitude pairs needs a little work.  In the examples, 'close' means the numbers are nearly the same, so the flight is not very accurate.  Also, the example doesn't check the velocity, so the drone tends to shoot through the target rather than stabilise at it.

  9. Send the drone back to land at its take-off location, by changing to `RTL` mode.

    > The example just stops after sending the `RTL` mode command.  

[Back to tutorial contents](README.md#contents)

## Exercises

All exercises work using the `old_school` code so run the simulation using `docker-compose -f docker-compose-old-school.yml up --build`.

1. Run the simulation and watch it progress using <a target="_blank" href="https://studio.foxglove.dev">https://studio.foxglove.dev</a>.  Use the teleoperation buttons (the little gamepad-like panels provided in the example layout, described in [Introspecting ROS](../README.md#introspecting-ros)) to interrupt the flight during climb or movement.  Play around with the teleoperation buttons to explore how they work, with careful attention to the relevant frame of reference.  Edit the panel settings and see what difference they make.

2. Interrupt a simulation again and add a `Publisher` panel in Foxglove.  Publish messages to the `/vehicle_1/mavros/setpoint_velocity/cmd_vel_unstamped` topic and see how the drone responds.

3. Interrupt a simulation and use a `Publisher` panel to send messages to the `/vehicle_1/mavros/setpoint_position/global` topic.  Try sending different latitude and longitude locations from the project briefing document.

    > *Note* due to a bug somewhere, you'll need to delete the `seq` line in the publisher panel to get this to work.  

4. Interrupt a simulation and use a combination of the gamepad and the setpoint publisher to land the drone at or near the target.  Use the `/vehicle_1/gimbal_tilt_cmd` publisher to move the camera, with `0` being horizontal and `1.57` being straight downwards.

[Back to tutorial contents](README.md#contents)

