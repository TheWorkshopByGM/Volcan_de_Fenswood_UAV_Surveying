Pre-requisites:
- docker
- vscode
- git

For Docker simulation on either Linux or Windows, navigate to cloned folder and run `docker-compose up --build`.

For native development on Linux:
- ROS2 Foxy: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
- Weird bug before ROS would work: pip install argcomplete 
- Colcon: https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html
- Create a workspace, as far as Step 3 (cloning) https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html
- Clone https://github.com/arthurrichards77/fenswood_volcano_template into ws/src
- Clone https://github.com/mavlink/mavros into ws/src
- Pick up again from Step 4 https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html
- MAVROS takes forever to build!  Got an error about libmavconn - wait and see if that's a problem