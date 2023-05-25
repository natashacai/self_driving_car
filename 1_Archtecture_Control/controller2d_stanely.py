import cutils
import numpy as np

'''
Pure pursuit controller: 

'''

class controller2d_pure_pursuit(object):

    def __init__(self, waypoints):
        ''' initialize the object '''
        self.vars = cutils.CUtils()
        self._current_x = 0  #current position in x direction
        self._current_y = 0  #current position in y direction
        self._current_yaw = 0 #current yaw angle
        self._current_speed = 0 #current speed
        self._desired_speed = 0 #desired speed
        self._current_frame = 0 #current frame per second
        self._current_timestamp = 0 #current timestamp
        self._start_control_loop = False #Start Closed Control Loop or not
        self._set_throttle = 0 #input U: a(throttle)
        self._set_brake = 0 #input U: a(brake)
        self._set_steer = 0 #input U: delta(steering angle)
        self._waypoints = waypoints  # waypoints of the desired trajectory
        self._conv_rad_to_steer = 180.0 / 70.0 / np.pi #convert radians to steering angle
        self._pi = np.pi    # pi
        self._2pi = 2 * np.pi  # 2pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        ''' update the vehicle's current state'''
        self._current_x = x
        self._current_y = y
        self._current_yaw = yaw
        self._current_speed = speed
        self._current_timestamp = timestamp
        self._current_frame = frame
        if self._current_frame:
            self._start_control_loop = True
        '''***** why start control loop when frame is on?'''

    def update_desired_speed(self):
        ''' update desired speed v from waypoints [x, y, v] '''
        min_idx = 0
        min_dist = float('inf')
        for i in range(self._waypoints):
            dx = self._waypoints[i][0] - self._current_x
            dy = self._waypoints[i][1] - self._current_y
            distance = np.linalg.norm(np.array([dx, dy]))
            if distance < min_dist:
                min_dist = distance
                min_idx = i
        if min_idx < len(self._waypoints) - 1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    '''
    def update_nearest_waypoints(self):
        dx = np.array(self._waypoints[:, :2] - self._current_x)
        dy = np.array(self._waypoints[:, :2] - self._current_y)
        distance = np.linalg.norm(np.array([dx, dy]))
    '''


    def update_waypoints(self, new_waypoints):
        ''' update waypoints from outside new waypoints'''
        self._waypoints = new_waypoints

    def get_commands(self):
        ''' output commands from the acuator '''
        return self._set_throttle, self._set_brake, self._set_steer

    def set_throttle(self, input_throttle):
        ''' update throttle position'''
        self._set_throttle = np.fmax(np.fmin(input_throttle, 1.0), 0)

    def set_brake(self, input_brake):
        ''' update brake position'''
        self._set_brake = np.fmax(np.fmin(input_brake, 1.0), 0)

    def set_steer(self, input_steer_in_rad):
        ''' set steering angle, Covnert radians to [-1, 1]'''
        input_steer = self._conv_rad_to_steer * input_steer_in_rad
        self._set_steer = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        ''' **** why 1.0 instead of 1.22? what is the unity of input steer'''

############################################################################################

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x = self._current_x
        y = self._current_y
        v = self._current_speed
        yaw = self._current_yaw
        self.update_desired_speed()
        v_desired_speed = self._desired_speed
        t = self._current_timestamp
        waypoints = self._waypoints
        throttle_output = 0
        brake_output = 0
        steer_output = 0

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
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('error_previous', 0.0)
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

            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.

            # PID controller for longitudinal control
            K_P = 0.2
            K_I = 0.05
            K_D = 0.01

            dt = t - self.vars.t_previous
            speed_error = self._current_speed - self._desired_speed

            proportion_term = K_P * speed_error
            integrator_term = K_I * (dt * speed_error + self.vars.integral_error_previous)
            derivative_term = K_D * ((speed_error - self.vars.integral_error_previous)/dt)
            desired_acceleration = proportion_term + integrator_term + derivative_term

            if desired_acceleration > 0:
                throttle_output = (np.tanh(desired_acceleration) + 1)/2
                if throttle_output - self.vars.throttle_previous > 0.1:
                    throttle_output = self.vars.throttle_previous + 0.1
            else:
                throttle_output = 0



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

            steer_output = 0

            # Stanely Controller for lateral control

            # Waypoint Path coefficients
            path_slope = (waypoints[-1][1] - waypoints[0][1])/(waypoints[-1][0] - waypoints[0][0])
            c = waypoints[0][1] - path_slope * waypoints[0][0]
            b = 1.0
            a = -path_slope

            # Heading Error
            heading_error = np.arctan((-a)/b) - yaw
            if heading_error > self._pi:
                heading_error -= self._2pi
            elif heading_error < -self._pi:
                heading_error += self._2pi

            # Cross-track error
            cross_track_error = (a * x + b * y + c) / np.linalg.norm(a, b)
            k_e = 0.25
            cross_track_term = np.arctan(k_e * cross_track_error / v)
            if cross_track_term > np.pi/2:
                cross_track_term = np.pi/2
            elif cross_track_term < - np.pi/2:
                cross_track_term = - np.pi/2

            # Change the steer output with the lateral controller.
            steer_output = heading_error + cross_track_term
            if steer_output >= 1.22:
                steer_output = 1.22
            elif steer_output <= -1.22:
                steer_output = -1.22

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)  # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)  # in percent (0 to 1)




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
        self.vars.integral_error_previous = integrator_term
        self.vars.throttle_previous = throttle_output
        self.vars.integral_error_previous = speed_error
        self.vars.t_previous = t


















