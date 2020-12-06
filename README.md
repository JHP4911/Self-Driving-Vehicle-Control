# Self-Driving-Vehicle-Control

## Project

The goal of this project is to control the vehicle to follow a race track by navigating through preset waypoints. The vehicle needs to reach these waypoints at certain desired speed & direction, so both longitudinal and lateral control is required.

I designed and implemented a controller for the CARLA simulator. For throttle and braking, PID controller (longitudinal vehicle control) is used and to keep car on race track, Stanley controller( lateral vehicle control) is used.  



## Outputs

The output of the controller is the vehicle throttle, brake and steering angle commands.

The throttle and brake are achieved from the Longitudinal speed control and the steering is achieved from the Lateral Control.

The outputs graph has been added to folder controller_folder.
