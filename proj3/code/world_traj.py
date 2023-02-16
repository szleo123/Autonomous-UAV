import numpy as np
from copy import deepcopy
from scipy.sparse.linalg import spsolve
from scipy.sparse import csr_matrix
from .graph_search import graph_search
from .occupancy_map import OccupancyMap

class WorldTraj(object):
    """

    """
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must choose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.19, 0.19, 0.19])
        self.margin = 0.58 # was .58

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)
        self.world = world
        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        self.points = np.zeros((1,3)) # shape=(n_pts,3)

        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # STUDENT CODE HERE
        self.speed = 2.8 # constant speed
        self.points = self.sparse_points(self.path)
        # used for toy test(use test_empty.json and uncomment the code below)
        self.times = self.compute_time(self.points)
        self.coef_x = self.calculate_coef(0).reshape((-1, 6))
        self.coef_y = self.calculate_coef(1).reshape((-1, 6))
        self.coef_z = self.calculate_coef(2).reshape((-1, 6))

    def calculate_coef(self, axis):
        n_inter = len(self.points) - 2
        mat = np.zeros((6*n_inter+6, 6*(n_inter+1))).astype(float)
        b = np.zeros(6*n_inter+6).astype(float)
        ######################################
        ### setting the end point conditions
        ######################################
        mat[0, 5] = 1
        mat[1, 4] = 1
        mat[2, 3] = 2
        end_t = self.times[-1] - self.times[-2]
        mat[3, -6:] = np.array([end_t**5, end_t**4, end_t**3, end_t**2, end_t, 1])
        mat[4, -6:] = np.array([5 * end_t ** 4, 4 * end_t ** 3, 3 * end_t ** 2, 2 * end_t, 1, 0])
        mat[5, -6:] = np.array([20 * end_t ** 3, 12 * end_t ** 2, 6 * end_t, 2, 0, 0])
        start_pt = self.points[0, axis]
        end_pt = self.points[-1, axis]
        b[0] = start_pt
        b[1] = 0  # velocity at the start point
        b[2] = 0  # acceleration at the start point
        b[3] = end_pt
        b[4] = 0  # velocity at the end point
        b[5] = 0  # acceleration at the end point
        #############################################
        ### setting the intermediate conditions
        ############################################
        for i in range(n_inter):
            t = self.times[i+1] - self.times[i]
            mat[6*(i+1), 6*i:6*i+6] = np.array([t**5, t**4, t**3, t**2, t, 1])
            mat[6*(i+1)+2, 6*i:6*i+6] = np.array([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0])
            mat[6*(i+1)+3, 6*i:6*i+6] = np.array([20*t**3, 12*t**2, 6*t, 2, 0, 0])
            mat[6*(i+1)+4, 6*i:6*i+6] = np.array([60*t**2, 24*t, 6, 0, 0, 0])
            mat[6*(i+1)+5, 6*i:6*i+6] = np.array([120*t, 24, 0, 0, 0, 0])
            mat[6*(i+1)+1, 6*(i+1)+5] = 1
            mat[6*(i+1)+2, 6*(i+1)+4] = -1
            mat[6*(i+1)+3, 6*(i+1)+3] = -2
            mat[6*(i+1)+4, 6*(i+1)+2] = -6
            mat[6*(i+1)+5, 6*(i+1)+1] = -24
            b[6*(i+1)] = self.points[i+1, axis]
            b[6*(i+1)+1] = self.points[i+1, axis]
        ###############################################
        ### solve for coefficients
        ###############################################
        smat = csr_matrix(mat)
        coef = spsolve(smat, b)
        return coef


    # def refine_path(self, path):
    #     if len(path) > 2:
    #         new_path = deepcopy(path)
    #         delete = []
    #         for i in range(len(path) - 2):
    #             p1 = path[i]
    #             p2 = path[i + 1]
    #             p3 = path[i + 2]
    #             side1 = p2 - p1
    #             side2 = p3 - p1
    #             area = 0.5 * np.linalg.norm(np.cross(side1, side2))
    #             if area == 0:
    #                 delete.append(i + 1)
    #         new_path = np.delete(new_path, delete, axis=0)
    #         new_path = self.sparse_points(new_path)
    #         return new_path
    #     return path

    def sparse_points(self, path):
        cur = 0
        keep_dist = 1
        n_points = len(path)
        delete = []
        while cur < n_points - 1:
            search = n_points - 1
            while search > cur:
                if len(self.world.path_collisions(path[[cur, search]], self.margin)) == 0:
                    checked_points = path[cur+1:search]
                    _, closest_dist = self.world.closest_points(checked_points)
                    # closest_dist_boundary = self.world.min_dist_boundary(checked_points)
                    # closest_dist = np.minimum(closest_dist_boundary, closest_dist_obstacles)
                    removed_index = np.where(closest_dist > keep_dist)[0] + cur + 1
                    delete = delete + removed_index.tolist()
                    cur = search
                else:
                    search -= 1
        delete = list(set(delete))

        new_path = np.delete(path, delete, axis=0)
        new_delete = set()
        old = 0
        cur = 1
        next = 2
        while next < len(new_path):
            dist1 = np.linalg.norm(new_path[cur] - new_path[old])
            dist2 = np.linalg.norm(new_path[cur] - new_path[next])
            min_dist = np.min([dist1, dist2])
            if min_dist < 0.5:
                if len(self.world.path_collisions(new_path[[old, next]], self.margin)) == 0:
                    new_delete.add(cur)
                    cur = next
                    next += 1
                else:
                    old = cur
                    cur = next
                    next += 1
            else:
                old = cur
                cur = next
                next += 1
        new_delete = list(new_delete)
        new_path = np.delete(new_path, new_delete, axis=0)
        kept = []
        for i in range(len(new_path)):
            kept.append(np.where(np.all(path == new_path[i], axis=1))[0].item())
        delete = list(set(range(len(path))) - set(kept))
        #############################################
        ### add necessary points back
        #############################################
        d = deepcopy(delete)
        new_path = np.delete(path, delete, axis=0)
        for i in d:
            dist = np.linalg.norm(new_path - path[i], axis=1)
            min_dist = np.min(dist)
            if min_dist > 1.5:
                delete.remove(i)
                new_path = np.delete(path, delete, axis=0)
        return new_path


    def compute_time(self, points):
        '''
        Compute the start time for each segment

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        Outputs:
            T, (N,) array of N-1 start time for each segment
        '''
        N = len(points)
        T = np.zeros(N)
        for i in range(N - 1):
            T[i+1] = np.linalg.norm(points[i+1] - points[i]) / self.speed
        for i in range(1, N-1):
            T[i] = T[i] * (np.linalg.norm(points[i+1] - points[i]) + np.linalg.norm(points[i] - points[i-1])) / np.linalg.norm(points[i+1] - points[i-1])
        T[1] = T[1] * 2
        T[-1] = T[-1] * 3
        T = np.cumsum(T)
        return T

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        cur_seg = -1
        for num_segment in range(len(self.times)):
            if t < self.times[num_segment]:
                cur_seg = num_segment - 1
                t = t - self.times[num_segment-1]
                break
        if cur_seg == -1:
            x = self.points[-1]
        else:
            t1 = np.array([t**5, t**4, t**3, t**2, t, 1])
            t2 = np.array([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0])
            t3 = np.array([20*t**3, 12*t**2, 6*t, 2, 0, 0])
            t4 = np.array([60*t**2, 24*t, 6, 0, 0, 0])
            t5 = np.array([120*t, 24, 0, 0, 0, 0])
            x[0] = self.coef_x[cur_seg] @ t1
            x[1] = self.coef_y[cur_seg] @ t1
            x[2] = self.coef_z[cur_seg] @ t1
            x_dot[0] = self.coef_x[cur_seg] @ t2
            x_dot[1] = self.coef_y[cur_seg] @ t2
            x_dot[2] = self.coef_z[cur_seg] @ t2
            x_ddot[0] = self.coef_x[cur_seg] @ t3
            x_ddot[1] = self.coef_y[cur_seg] @ t3
            x_ddot[2] = self.coef_z[cur_seg] @ t3
            x_dddot[0] = self.coef_x[cur_seg] @ t4
            x_dddot[1] = self.coef_y[cur_seg] @ t4
            x_dddot[2] = self.coef_z[cur_seg] @ t4
            x_ddddot[0] = self.coef_x[cur_seg] @ t5
            x_ddddot[1] = self.coef_y[cur_seg] @ t5
            x_ddddot[2] = self.coef_z[cur_seg] @ t5


        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output

