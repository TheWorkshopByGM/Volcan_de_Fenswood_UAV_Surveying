# Getting Started

[Back to tutorial contents](README.md#contents)

## Prerequisites

You will need to install [git](https://git-scm.com/downloads) to access the software and [docker](https://docs.docker.com/get-docker/) to run it.  These are supported on Windows, Mac and Linux.  I also recommend [Visual Studio Code](https://code.visualstudio.com/) as an editing environment with easy access to terminal windows.

## Getting the template

Navigate to a suitable file folder on your computer and run
```
git clone https://github.com/StarlingUAS/fenswood_volcano_template.git
```

## Running a first simulation
 
For Docker simulation on either Linux or Windows, open a terminal, navigate to your cloned folder and run `docker-compose up --build`.

> This could take a very long time on your first attempt, perhaps as much as half an hour.  It has a lot of stuff to download.

You should eventually see an awful lot of stuff flying by in the terminal window.  The excerpt below is just a short example.
```
controller_1       | [controller-1] [INFO] [1641487809.030179200] [vehicle_1.example_controller]: Arm request sent
controller_1       | [controller-1] [INFO] [1641487809.031134000] [vehicle_1.example_controller]: Controller state: arming for 13 steps
mavros_1           | [run_ros1.sh-1] [ERROR] [1641487809.032953700]: FCU: PreArm: Bad GPS Position     
controller_1       | [image_processor-2] [INFO] [1641487809.057483100] [vehicle_1.image_processor]: Got an image of 480 x 640
```

### Watching it fly

If that's happening, open a web browser and open <a target="_blank" href="http://localhost:8080">http://localhost:8080</a>.  You should see a Gazebo window with a green field, a couple of cones, and a drone.  Watch long enough and the drone should take-off.  Watch even longer and it should return.

### Introspecting ROS

Open another web browser and navigate to <a target="_blank" href="https://studio.foxglove.dev">https://studio.foxglove.dev</a>.  Select `Open Connection` and enter `ws://localhost:9090` in the URL box, if it's not already there.  This gets you into Foxglove, a powerful dashboard for ROS.  To get started, click the `Layouts` button (second down on the left hand menu) and then the `Import Layout` button at the top of the panel that opens.  Navigate to your `fenswood_volcano_template` folder and select `fenswood_example.json`.  Click the `Layouts` button again to close the panel and maximize the rest of the dashboard.  You should see logs, drone status and position, and the drone camera feed, among other things.

[Back to tutorial contents](README.md#contents)

## Exercise

Have a play around.  Watch flights through both Gazebo and Foxglove and explore their interfaces.  When finished, stop everything with `Ctrl+C` in the terminal where you ran `docker-compose`.