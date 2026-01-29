# Unitree Low Level

This project provides a low-level controller for Unitree Go2/G1 robots.

## Installation

It is recommended to install via [unitree_ws](https://github.com/Renkunzhao/unitree_ws.git), which provides a Dockerfile and helper scripts for an easier setup.

## Features

- Acts as a hardware interface: reads sensor data, calls the high-level controller, and sends commands to the robot.
- Emergency stop (E-stop) support.
- Provides IK + PD control for robot initialization.

## Usage

For a demonstration, see [legged_rl_deploy](https://github.com/Renkunzhao/legged_rl_deploy.git).

