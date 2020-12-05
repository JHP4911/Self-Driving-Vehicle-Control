#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        # update latest state feedback from the Carla Simulator
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        # calculate the desired speed for the controller to track the waypoints
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        # update the current set of waypoints for the controller to navigate
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # set the throttle command for tyhr CARLA simulator server.
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # set the steer command for the CARLA and Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # set the brake for CARLA and Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):  
        #function used to implement the controller
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        # MODULE 7: DECLARE USAGE VARIABLES HERE
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('error_previous', 0.0)
        self.vars.create_var('integral_error_previous', 0.0)
        self.vars.create_var('throttle_previous', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                                    
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """
            # IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            #tuning the gains
            kp = 1.0
            ki = 0.2
            kd = 0.01

            throttle_output = 0
            brake_output    = 0

            # pid control = current time - previous time
            st = t - self.vars.t_previous

            # error term = desired velocity-current velocity
            e_v = v_desired - v

            # Integral value= previous error+ e*de/dt
            inte_v = self.vars.integral_error_previous + e_v * st

            # Derivative value = de(t)/dt 
            derivate = (e_v - self.vars.error_previous) / st

            #longitudinal control(acceleration)
            acc = kp * e_v + ki * inte_v + kd * derivate

            if acc > 0:
                throttle_output = np.tanh(acc)
                # to get output in limit (0,1)
                throttle_output = max(0.0, min(1.0, throttle_output))
                if throttle_output - self.vars.throttle_previous > 0.1:
                    throttle_output = self.vars.throttle_previous + 0.1
            else:
                throttle_output = 0

            # IMPLEMENTATION OF LATERAL CONTROLLER HERE

            # Change the steer output with the lateral controller. 
            steer_output    = 0
            # Use stanley controller for lateral control
            k_e = 0.3
            slope = (waypoints[-1][1]-waypoints[0][1])/ (waypoints[-1][0]-waypoints[0][0])
            a = -slope
            b = 1.0
            c = (slope*waypoints[0][0]) - waypoints[0][1]

            # heading error
            yaw_path = np.arctan2(waypoints[-1][1]-waypoints[0][1], waypoints[-1][0]-waypoints[0][0])
            # yaw_path = np.arctan2(slope, 1.0)  # This was turning the vehicle only to the right (some error)
            yaw_diff_heading = yaw_path - yaw
            #if heading error is large, the car will steer in the opposite direction to correct the heading error
            if yaw_diff_heading > np.pi:
                yaw_diff_heading -= 2 * np.pi
            if yaw_diff_heading < - np.pi:
                yaw_diff_heading += 2 * np.pi

            # crosstrack error
            current_xy = np.array([x, y])
            crosstrack_error = np.min(np.sum((current_xy - np.array(waypoints)[:, :2])**2, axis=1))
            yaw_cross_track = np.arctan2(y-waypoints[0][1], x-waypoints[0][0])
            yaw_path2ct = yaw_path - yaw_cross_track
            #if crosstalk error is large then the vehicle will run towards the path in bad way
            if yaw_path2ct > np.pi:
                yaw_path2ct -= 2 * np.pi
            if yaw_path2ct < - np.pi:
                yaw_path2ct += 2 * np.pi
            if yaw_path2ct > 0:
                crosstrack_error = abs(crosstrack_error)
            else:
                crosstrack_error = - abs(crosstrack_error)
            yaw_diff_crosstrack = np.arctan(k_e * crosstrack_error / (v))

            # final expected steering
            steer_expect = yaw_diff_crosstrack + yaw_diff_heading
            if steer_expect > np.pi:
                steer_expect -= 2 * np.pi
            if steer_expect < - np.pi:
                steer_expect += 2 * np.pi
            steer_expect = min(1.22, steer_expect)
            steer_expect = max(-1.22, steer_expect)

            #update
            steer_output = steer_expect

            # SET CONTROLS OUTPUT

            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        # STORE OLD VALUES HERE FOR NEXT ITTERATION 

        self.vars.v_previous = v
        # Store forward speed to be used in next step
        self.vars.throttle_previous = throttle_output
        self.vars.t_previous = t
        self.vars.error_previous = e_v
        self.vars.integral_error_previous = inte_v

