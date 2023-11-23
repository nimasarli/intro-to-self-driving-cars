#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np
from loguru import logger as log
import sys

log.remove()
log.add(sys.stdout, level="ERROR") # change to DEBUG to print debugs

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
        self._kp                 = 2.0 # 5.0
        self._ki                 = 0.5
        self._kd                 = 0.1
        self._stanley_gain       = 1 # 0.1
        self._stanley_softening_const = 0.01
        self._pure_pursuit_kdd    = 1.0
        self._l_r                  = 1.5
        self._l_f                  = 1.5

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
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
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to degree and normalize by dividing with 70 degrees (max steering angle)
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds [-1, 1] (1 means 100%)
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def high_level_PID(self, v, v_desired):
        """
        High-level PID controller for longitudinal control.
        Takes desired speed and generates desired acceleration.
        Desired acceleration should be fed into low-level controller 
        or used in conjunction with a feed-forward control to find throttle
        values. 
        In this porject, outpur of PID is directly taken as throttle values
        """
        error = v_desired - v
        sample_time = self._current_timestamp - self.vars.prev_timestamp
        
        p_term = self._kp * error
        i_term = self._ki * error * sample_time + self.vars.error_integral_prev
        d_term = self._kd * (error - self.vars.error_prev) / sample_time
        
        # Updates for next tick
        self.vars.error_integral_prev = i_term
        # self.vars.error_prev = error
        
        xddt_des = p_term + i_term + d_term
        return xddt_des        
        
    def steer_to_angle(self, steer_percent):
        """Convert steer value [-1, 1] to angle in radians. 1 is mapped to 70 degrees

        Args:
            steer_percent (double): steer in [-1, 1]
        """
        return steer_percent / self._conv_rad_to_steer

    def compute_lateral_errors(self):
        """ Computes heading error and cross-track error 
        heading error: desired heading (tangent slope at nearest desired waypoint) - vehicle heading (yaw)
        cross-track error: is the distance from center of front axle to
        the closest point on the path 
        """
        # Find crosstrack error magnitude 
        cross_error_magnitude = float("inf")
        min_idx = 0
        for i in range(len(self._waypoints)):
            x_f = self._current_x + self._l_f * np.cos(self._current_yaw)
            y_f = self._current_y + self._l_f * np.sin(self._current_yaw)
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - x_f,
                    self._waypoints[i][1] - y_f]))
            if dist < cross_error_magnitude:
                cross_error_magnitude = dist
                min_idx = i
        
        # Find crosstrack error sign by finding the angle b/w distance vector (pointing from front wheel axle
        # to waypoint) and a vector orthogonal to vehicle bearing direction (thus pointing at angle psi+90 deg)
        # 0 < angle < pi/2 --> cross-track error > 0
        # pi/2 < angle < pi --> cross-track error < 0 
        dist_vec = np.array([self._waypoints[min_idx][0] - x_f, self._waypoints[min_idx][1] - y_f])
        ortho_vec = np.array([np.cos(self._current_yaw + self._pi / 2.), np.sin(self._current_yaw + self._pi / 2.)])
        cross_error_sign = np.dot(dist_vec, ortho_vec) 
        
        cross_error = cross_error_sign * cross_error_magnitude
        
        desired_heading = np.arctan2(self._waypoints[min_idx][1] - self._waypoints[min_idx-1][1],  
                                    self._waypoints[min_idx][0] - self._waypoints[min_idx-1][0]) # use of atan2 instead of atan is crucial, because 
        # atan is [-pi/2, pi/2] and it can jump b/w pi/2 and -pi/2, however atan2 is [-pi, pi] (does it jump b/w -pi and pi?)
        
        heading_error = desired_heading - self._current_yaw
        log.debug(f"y, y_prev, x, x_prev={self._waypoints[min_idx][1]},{self._waypoints[min_idx-1][1]},{self._waypoints[min_idx][0]},{self._waypoints[min_idx-1][0]}")
        log.debug(f"desired_heading[deg]={np.rad2deg(desired_heading)}, current_yaw[deg]={np.rad2deg(self._current_yaw)}")
        log.debug(f"heading_error[deg]={np.rad2deg(heading_error)}, crosstrack_error[m]={np.rad2deg(cross_error)}")
        
        return heading_error, cross_error
        
    def stanley_controller(self, v):
        prev_steer_angle = self.steer_to_angle(self.vars.prev_set_steer)
        slip_angle = np.arctan(self._l_r * np.tan(prev_steer_angle) / (self._l_r + self._l_f))
        v_f = v * np.cos(slip_angle) / np.cos(prev_steer_angle) # calculated by 
        # assuming velocity component along vehicle length is the same everywhere
        # also assuming zero front tire slip angles 
        log.debug(f"prev_steer_angle[deg]={np.rad2deg(prev_steer_angle)}, slip_angle[deg]={np.rad2deg(slip_angle)}, v_f={v_f}, v={v}")
        
        heading_error, crosstrack_error = self.compute_lateral_errors()
        crosstrack_error_comp = np.arctan(self._stanley_gain * crosstrack_error / (self._stanley_softening_const + v_f))
        delta = heading_error + crosstrack_error_comp
        log.debug(f"Stanley output (before sat) -- delta[deg]={np.rad2deg(delta)}, 1st term[deg]={np.rad2deg(heading_error)}\
            , 2nd term[deg]={np.rad2deg(crosstrack_error_comp)}")
        log.debug("--------------------")
        return delta
        
    def find_look_ahead_idx(self, v):
        x_r = self._current_x - self._l_r * np.cos(self._current_yaw)
        y_r = self._current_y - self._l_r * np.sin(self._current_yaw)
        def closest_idx_on_trajectory():
            dx = [x for x,_,_ in self._waypoints] - x_r
            dy = [y for _,y,_ in self._waypoints] - x_r
            return np.argmin(np.hypot(dx, dy))

        def look_ahead_idx_from(closest_index):
            l_d = self._pure_pursuit_kdd * v
            look_ahead_idx = closest_index
            dist_from_rear_axle = 0
            while l_d > dist_from_rear_axle:
                if (look_ahead_idx + 1) >= len(self._waypoints):
                    break  # not exceed goal
                look_ahead_idx += 1
                dist_from_rear_axle = np.linalg.norm(np.array([self._waypoints[look_ahead_idx][0] - x_r, \
                self._waypoints[look_ahead_idx][1] - y_r]))
            return look_ahead_idx

        return look_ahead_idx_from(closest_idx_on_trajectory())
    
    def pure_pursuit_controller(self, v):
        """Pure pursuit lateral controller
        """
        look_ahead_idx = self.find_look_ahead_idx(v)
        if look_ahead_idx < len(self._waypoints):
            t_x = self._waypoints[look_ahead_idx][0]
            t_y = self._waypoints[look_ahead_idx][1]
        else:  # toward goal
            t_x = self._waypoints[-1][0]
            t_y = self._waypoints[-1][1]
            look_ahead_idx = len(self._waypoints) - 1
            
        # Look-ahead angle 
        x_r = self._current_x - self._l_r * np.cos(self._current_yaw)
        y_r = self._current_y - self._l_r * np.sin(self._current_yaw)
        alpha = np.arctan2(t_y - y_r, t_x - x_r) - self._current_yaw

        delta = np.arctan(2. * (self._l_r+self._l_f) * np.sin(alpha) / (self._pure_pursuit_kdd * v + 0.0001))
   
        return delta
    
    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
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

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('prev_timestamp', 0.0)
        self.vars.create_var('error_integral_prev', 0.0)
        self.vars.create_var('error_prev', 0.0)
        self.vars.create_var('prev_set_steer', 0.0)

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
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            # High-level PID controller
            # For this project, low-level controller is assumed to be identity i.e. desired acceleration 
            # is directly fed as desired throttle
            xddt_desired = self.high_level_PID(v, v_desired)

            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            if xddt_desired >= 0.:
                throttle_output = xddt_desired
                brake_output    = 0.
            else:
                throttle_output = 0.
                brake_output = -xddt_desired

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            delta_desired = self.stanley_controller(v)
            #delta_desired = self.pure_pursuit_controller(v)
            
            # Change the steer output with the lateral controller. 
            steer_output = delta_desired

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in percent (-1 to 1), steer_output is in rad
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.error_prev = v_desired - v
        self.vars.prev_set_steer = self._set_steer
        self.vars.prev_timestamp = self._current_timestamp
