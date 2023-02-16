import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2
        self.K_d = np.eye(3) * 5
        # self.K_d[2,2] = 1.2
        self.K_p = np.eye(3) * 10
        # self.K_p[2,2] = 0.15
        self.K_R = np.eye(3) * 800
        self.K_w = np.eye(3) * 27
        self.rotation = None
        db = self.arm_length * self.k_thrust
        self.A = np.array([[self.k_thrust, self.k_thrust, self.k_thrust, self.k_thrust],
                           [0, db, 0, -db],
                           [-db, 0, db, 0],
                           [self.k_drag, -self.k_drag, self.k_drag, -self.k_drag]])

        # STUDENT CODE HERE

    def calculate_F_des(self, a_T, v, v_T, r, r_T):
        a_des = a_T - self.K_d @ (v - v_T) - self.K_p @ (r - r_T)
        F_des = self.mass * a_des + np.array([0, 0, self.mass * self.g])
        return F_des

    def calculate_u1(self, F_des):
        R = self.rotation.as_matrix()
        b3 = R @ np.array([0, 0, 1])
        u1 = b3.dot(F_des)
        return u1

    def calculate_R_des(self, F_des, angle):
        a = np.array([np.cos(angle), np.sin(angle), 0])
        b3 = F_des / np.linalg.norm(F_des)
        b2 = np.cross(b3, a)
        b2 = b2 / np.linalg.norm(b2)
        b1 = np.cross(b2, b3)
        R_des = np.zeros((3,3))
        R_des[:, 0] = b1
        R_des[:, 1] = b2
        R_des[:, 2] = b3
        return R_des

    def calculate_e_R(self, R_des):
        R = self.rotation.as_matrix()
        mat = 0.5 * (R_des.T @ R - R.T @ R_des)
        e_R = np.array([mat[2, 1], -mat[2, 0], mat[1,0]])
        return e_R

    def calculate_u2(self, e_R, w):
        e_w = w
        u2 = self.inertia @ (-self.K_R @ e_R - self.K_w @ e_w)
        return u2

    def calculate_motor_speed(self, u1, u2):
        U = np.zeros(4)
        U[0] = u1
        U[1:] = u2
        omega_2 = np.linalg.inv(self.A) @ U
        omega_2[omega_2 < 0] = 0 # a trick
        omega = np.sqrt(omega_2)
        return omega

    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE
        self.rotation = Rotation.from_quat(state['q'])
        a_T = flat_output['x_ddot']
        v = state['v']
        v_T = flat_output['x_dot']
        r = state['x']
        r_T = flat_output['x']
        F_des = self.calculate_F_des(a_T, v, v_T, r, r_T)
        angle = flat_output['yaw']
        R_des = self.calculate_R_des(F_des, angle)
        e_R = self.calculate_e_R(R_des)
        w = state['w']
        u1 = self.calculate_u1(F_des)
        u2 = self.calculate_u2(e_R, w)
        cmd_motor_speeds = self.calculate_motor_speed(u1, u2)

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
        return control_input
