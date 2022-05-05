# Volcan de Fenswood UAV Surveying 

This repository provides a UAV volcano sruveying simulation based of the Starling UAS project, which provides a docker application which allows for easy development of UAV simulations and control supported by a ROS2 framework. 

## Starling UAS Tutorials

The tutorial is hosted here: https://starlinguas.github.io/fenswood_volcano_template/

But also, see [tutorial folder](tutorial) for detailed instructions and code walkthroughs. 

## Quick Start

 - Install [git](https://git-scm.com/downloads)
 - Install [docker](https://docs.docker.com/get-docker/) to run it.
 - Clone this repo `git clone https://github.com/arthurrichards77/fenswood_volcano_template`
 - Run `docker-compose up --build` in the `fenswood_volcano_template` folder.
 - Watch the Gazebo rendering on [http://localhost:8080](http://localhost:8080)
 - Interact using [https://studio.foxglove.dev](https://studio.foxglove.dev) connected via `ws://localhost:9090`
