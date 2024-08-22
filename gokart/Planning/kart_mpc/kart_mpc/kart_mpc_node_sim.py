#!/usr/bin/env python3
import math
from dataclasses import dataclass, field

import cvxpy
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.linalg import block_diag
from scipy.sparse import block_diag, csc_matrix
from sensor_msgs.msg import LaserScan
import csv
import os
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
import yaml

@dataclass
class mpc_config:
    NXK: int = 4  # length of kinematic state vector: z = [x, y, v, yaw]
    NU: int = 2  # length of input vector: u = = [steering speed, acceleration]
    TK: int = 8  # finite time horizon length kinematic

    Rk: list = field(default_factory=lambda: np.diag([0.01, 100.0]))  
    Rdk: list = field(default_factory=lambda: np.diag([0.01, 100.0])) 
    Qk: list = field(default_factory=lambda: np.diag([13.5, 13.5, 5.5, 13.0]))  
    Qfk: list = field(default_factory=lambda: np.diag([13.5, 13.5, 5.5, 13.0]))

    N_IND_SEARCH: int = 20
    DTK: float = 0.1  
    dlk: float = 0.12  
    LENGTH: float = 0.58  
    WIDTH: float = 0.31  
    WB: float = 0.33  
    MIN_STEER: float = -0.4189  
    MAX_STEER: float = 0.4189  
    MAX_DSTEER: float = np.deg2rad(180.0)
    MAX_SPEED: float = 6.0  
    MIN_SPEED: float = 0.0  
    MAX_ACCEL: float = 3.0  

@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    delta: float = 0.0
    v: float = 0.0
    yaw: float = 0.0

class MPC(Node):
    def __init__(self):
        super().__init__('kart_mpc_node')

        self.odom_sub = self.create_subscription(Odometry, '/kart/odom', self.pose_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/kart/ackermann_cmd', 10)

        self.viz_waypoints_pub = self.create_publisher(MarkerArray, '/kart/viz/waypoints', 10)
        self.viz_target_pub = self.create_publisher(Marker, '/kart/viz/target_point', 10)
        self.viz_ref_traj_pub = self.create_publisher(MarkerArray, '/kart/viz/ref_traj', 10)

        folder_path = '/home/autoware/gokart-autoware/src/universe/autoware.universe/gokart/configs/purdue_2024'
        filename = 'wp_sim_mpc.csv'
        self.csv_file_path = os.path.join(folder_path, filename)
        
        self.get_logger().info("Loading waypoints from {}".format(self.csv_file_path))
        self.waypoints = np.loadtxt(self.csv_file_path, delimiter=',', skiprows=1)
        self.waypoints = np.vstack((
            self.waypoints[:, 0],  
            self.waypoints[:, 1],  
            self.waypoints[:, 2], 
            self.waypoints[:, 3]   
        )).T

        self.visualize_waypoints(self.waypoints)

        self.config = mpc_config()
        self.odelta_v = None
        self.odelta = None
        self.oa = None
        self.prev_state = None
        self.prev_speed_output = None

        self.mpc_prob_init()

    def pose_callback(self, pose_msg):
        vehicle_state = State()
        vehicle_state.x = pose_msg.pose.pose.position.x
        vehicle_state.y = pose_msg.pose.pose.position.y
        quat = pose_msg.pose.pose.orientation

        quat = [quat.x, quat.y, quat.z, quat.w]
        euler = euler_from_quaternion(quat)
        yaw = euler[2]

        if self.prev_state is not None:
            if yaw - self.prev_state.yaw > 4.5:
                yaw -= 2*np.pi
            if yaw - self.prev_state.yaw < -4.5:
                yaw += 2*np.pi

        vehicle_state.yaw = yaw
        vehicle_state.v = self.prev_speed_output if self.prev_speed_output is not None else 0.0

        ref_path = self.calc_ref_trajectory(vehicle_state, self.waypoints[:, 0], self.waypoints[:, 1], self.waypoints[:, 2], self.waypoints[:, 3])
        x0 = [vehicle_state.x, vehicle_state.y, vehicle_state.v, vehicle_state.yaw]

        self.oa, self.odelta_v, ox, oy, oyaw, ov, state_predict = self.linear_mpc_control(ref_path, x0, self.oa, self.odelta_v)

        if self.oa is None or self.odelta_v is None:
            self.prev_state = None
            return

        steer_output = self.odelta_v[0]
        speed_output = vehicle_state.v + self.oa[0] * self.config.DTK

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steer_output
        drive_msg.drive.speed = speed_output * 2.0
        drive_msg.drive.acceleration = 0.01
        print(f"steer: {steer_output}, speed: {speed_output}, yaw: {vehicle_state.yaw}")
        self.drive_pub.publish(drive_msg)

        self.prev_state = vehicle_state
        self.prev_speed_output = speed_output

    def mpc_prob_init(self):
        self.xk = cvxpy.Variable((self.config.NXK, self.config.TK + 1))
        self.uk = cvxpy.Variable((self.config.NU, self.config.TK))

        objective = 0.0
        constraints = []

        self.x0k = cvxpy.Parameter((self.config.NXK,))
        self.x0k.value = np.zeros((self.config.NXK,))

        self.ref_traj_k = cvxpy.Parameter((self.config.NXK, self.config.TK + 1))
        self.ref_traj_k.value = np.zeros((self.config.NXK, self.config.TK + 1))

        R_block = block_diag(tuple([self.config.Rk] * self.config.TK))
        Rd_block = block_diag(tuple([self.config.Rdk] * (self.config.TK - 1)))
        Q_block = [self.config.Qk] * (self.config.TK)
        Q_block.append(self.config.Qfk)
        Q_block = block_diag(tuple(Q_block))

        objective += cvxpy.quad_form(cvxpy.vec(self.uk), R_block)
        error = self.xk - self.ref_traj_k
        objective += cvxpy.quad_form(cvxpy.vec(error), Q_block)
        objective += cvxpy.quad_form(cvxpy.vec(cvxpy.diff(self.uk, axis=1)), Rd_block)

        A_block = []
        B_block = []
        C_block = []
        path_predict = np.zeros((self.config.NXK, self.config.TK + 1))
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(path_predict[2, t], path_predict[3, t], 0.0)
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        m, n = A_block.shape
        self.Annz_k = cvxpy.Parameter(A_block.nnz)
        data = np.ones(self.Annz_k.size)
        rows = A_block.row * n + A_block.col
        cols = np.arange(self.Annz_k.size)
        Indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Annz_k.size))
        self.Annz_k.value = A_block.data
        self.Ak_ = cvxpy.reshape(Indexer @ self.Annz_k, (m, n), order="C")

        m, n = B_block.shape
        self.Bnnz_k = cvxpy.Parameter(B_block.nnz)
        data = np.ones(self.Bnnz_k.size)
        rows = B_block.row * n + B_block.col
        cols = np.arange(self.Bnnz_k.size)
        Indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Bnnz_k.size))
        self.Bk_ = cvxpy.reshape(Indexer @ self.Bnnz_k, (m, n), order="C")
        self.Bnnz_k.value = B_block.data

        self.Ck_ = cvxpy.Parameter(C_block.shape)
        self.Ck_.value = C_block

        constraints += [cvxpy.vec(self.xk[:, 1:]) == self.Ak_ @ cvxpy.vec(self.xk[:, :-1]) + self.Bk_ @ cvxpy.vec(self.uk) + (self.Ck_)]
        constraints += [cvxpy.abs(cvxpy.diff(self.uk[1, :]))/self.config.DTK<=self.config.MAX_DSTEER]
        constraints += [self.xk[:, 0]==self.x0k]
        constraints += [self.xk[2, :] <= self.config.MAX_SPEED]
        constraints += [self.xk[2, :] >= self.config.MIN_SPEED]
        constraints += [self.uk[0, :] <= self.config.MAX_ACCEL]
        constraints += [self.uk[0, :] >= -self.config.MAX_ACCEL]
        constraints += [self.uk[1, :] <= self.config.MAX_STEER]
        constraints += [self.uk[1, :] >= self.config.MIN_STEER]

        self.MPC_prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)

    def nearest_point(self, point, trajectory):
        """
        Return the nearest point along the given piecewise linear trajectory.
        Args:
            point (numpy.ndarray, (2, )): (x, y) of current pose
            trajectory (numpy.ndarray, (N, 2)): array of (x, y) trajectory waypoints
                NOTE: points in trajectory must be unique. If they are not unique, a divide by 0 error will destroy the world
        Returns:
            nearest_point (numpy.ndarray, (2, )): nearest point on the trajectory to the point
            nearest_dist (float): distance to the nearest point
            t (float): nearest point's location as a segment between 0 and 1 on the vector formed by the closest two points on the trajectory. (p_i---*-------p_i+1)
            i (int): index of nearest point in the array of trajectory waypoints
        """
        diffs = trajectory[1:,:] - trajectory[:-1,:]
        l2s   = diffs[:,0]**2 + diffs[:,1]**2
        dots = np.empty((trajectory.shape[0]-1, ))
        for i in range(dots.shape[0]):
            dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
        t = dots / l2s
        t[t<0.0] = 0.0
        t[t>1.0] = 1.0
        projections = trajectory[:-1,:] + (t*diffs.T).T
        dists = np.empty((projections.shape[0],))
        for i in range(dists.shape[0]):
            temp = point - projections[i]
            dists[i] = np.sqrt(np.sum(temp*temp))
        min_dist_segment = np.argmin(dists)
        return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment

    def calc_ref_trajectory(self, state, cx, cy, cyaw, sp):
        ref_traj = np.zeros((self.config.NXK, self.config.TK + 1))
        ncourse = len(cx)

        _, _, _, ind = self.nearest_point(np.array([state.x, state.y]), np.array([cx, cy]).T)

        ref_traj[0, 0] = cx[ind]
        ref_traj[1, 0] = cy[ind]
        ref_traj[2, 0] = sp[ind]
        ref_traj[3, 0] = cyaw[ind]

        travel = abs(state.v) * self.config.DTK
        dind = travel / self.config.dlk
        ind_list = int(ind) + np.insert(np.cumsum(np.repeat(dind, self.config.TK)), 0, 0).astype(int)
        ind_list[ind_list >= ncourse] -= ncourse
        ref_traj[0, :] = cx[ind_list]
        ref_traj[1, :] = cy[ind_list]
        ref_traj[2, :] = sp[ind_list]
        cyaw[cyaw - state.yaw > 4.5] = np.abs(
            cyaw[cyaw - state.yaw > 4.5] - (2 * np.pi)
        )
        cyaw[cyaw - state.yaw < -4.5] = np.abs(
            cyaw[cyaw - state.yaw < -4.5] + (2 * np.pi)
        )
        ref_traj[3, :] = cyaw[ind_list]

        self.visualize_ref_traj(ref_traj)
        return ref_traj

    def predict_motion(self, x0, oa, od, xref):
        path_predict = xref * 0.0
        for i, _ in enumerate(x0):
            path_predict[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, self.config.TK + 1)):
            state = self.update_state(state, ai, di)
            path_predict[0, i] = state.x
            path_predict[1, i] = state.y
            path_predict[2, i] = state.v
            path_predict[3, i] = state.yaw

        return path_predict

    def update_state(self, state, a, delta):
        if delta >= self.config.MAX_STEER:
            delta = self.config.MAX_STEER
        elif delta <= -self.config.MAX_STEER:
            delta = self.config.MAX_STEER

        state.x = state.x + state.v * math.cos(state.yaw) * self.config.DTK
        state.y = state.y + state.v * math.sin(state.yaw) * self.config.DTK
        state.yaw = state.yaw + (state.v / self.config.WB) * math.tan(delta) * self.config.DTK
        state.v = state.v + a * self.config.DTK

        if state.v > self.config.MAX_SPEED:
            state.v = self.config.MAX_SPEED
        elif state.v < self.config.MIN_SPEED:
            state.v = self.config.MIN_SPEED

        return state

    def get_model_matrix(self, v, phi, delta):
        A = np.zeros((self.config.NXK, self.config.NXK))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.config.DTK * math.cos(phi)
        A[0, 3] = -self.config.DTK * v * math.sin(phi)
        A[1, 2] = self.config.DTK * math.sin(phi)
        A[1, 3] = self.config.DTK * v * math.cos(phi)
        A[3, 2] = self.config.DTK * math.tan(delta) / self.config.WB

        B = np.zeros((self.config.NXK, self.config.NU))
        B[2, 0] = self.config.DTK
        B[3, 1] = self.config.DTK * v / (self.config.WB * math.cos(delta) ** 2)

        C = np.zeros(self.config.NXK)
        C[0] = self.config.DTK * v * math.sin(phi) * phi
        C[1] = -self.config.DTK * v * math.cos(phi) * phi
        C[3] = -self.config.DTK * v * delta / (self.config.WB * math.cos(delta) ** 2)

        return A, B, C
    
    def mpc_prob_solve(self, ref_traj, path_predict, x0):
        self.x0k.value = x0

        A_block = []
        B_block = []
        C_block = []
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(
                path_predict[2, t], path_predict[3, t], 0.0
            )
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        self.Annz_k.value = A_block.data
        self.Bnnz_k.value = B_block.data
        self.Ck_.value = C_block

        self.ref_traj_k.value = ref_traj

        # Solve the optimization problem in CVXPY
        # Solver selections: cvxpy.OSQP; cvxpy.GUROBI
        self.MPC_prob.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)

        if (
            self.MPC_prob.status == cvxpy.OPTIMAL
            or self.MPC_prob.status == cvxpy.OPTIMAL_INACCURATE
        ):
            ox = np.array(self.xk.value[0, :]).flatten()
            oy = np.array(self.xk.value[1, :]).flatten()
            ov = np.array(self.xk.value[2, :]).flatten()
            oyaw = np.array(self.xk.value[3, :]).flatten()
            oa = np.array(self.uk.value[0, :]).flatten()
            odelta = np.array(self.uk.value[1, :]).flatten()

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov
    
    def linear_mpc_control(self, ref_path, x0, oa, od):
        """
        MPC contorl with updating operational point iteraitvely
        :param ref_path: reference trajectory in T steps
        :param x0: initial state vector
        :param oa: acceleration of T steps of last time
        :param od: delta of T steps of last time
        """

        if oa is None or od is None:
            oa = [0.0] * self.config.TK
            od = [0.0] * self.config.TK

        # Call the Motion Prediction function: Predict the vehicle motion for x-steps
        path_predict = self.predict_motion(x0, oa, od, ref_path)
        poa, pod = oa[:], od[:]

        # Run the MPC optimization: Create and solve the optimization problem
        mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v = self.mpc_prob_solve(
            ref_path, path_predict, x0
        )

        return mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v, path_predict
    
    def visualize_waypoints(self, waypoints):
        """
        waypoints: np.array, first two columns are x, y positions
        """
        marker_array = MarkerArray()
        for i, waypoint in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.150
            marker.scale.y = 0.15
            marker.scale.z = 0.5
            marker.color.a = 1.0  # Alpha is set to 1 to ensure the marker is not transparent
            marker.color.r = 1.0  
            marker.color.g = 1.0
            marker.color.b = 1.0  
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0.0
            # marker.lifetime = rclpy.duration.Duration()  # Setting to zero so it never auto-deletes
            marker_array.markers.append(marker)

        self.viz_waypoints_pub.publish(marker_array)

    def visualize_target_point(self, waypoints, target_point):
        """
        waypoints: np.array, first two columns are x, y positions
        target_point: int, index of the target waypoint
        """
        waypoint = waypoints[target_point]
        target_marker = Marker()
        target_marker.header.frame_id = "map"
        target_marker.id = 0
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.scale.x = 0.15
        target_marker.scale.y = 0.15
        target_marker.scale.z = 0.5
        target_marker.color.a = 1.0  # Alpha is set to 1 to ensure the marker is not transparent
        target_marker.color.r = 255.0 
        target_marker.color.g = 0.0
        target_marker.color.b = 0.0 
        target_marker.pose.orientation.w = 1.0
        target_marker.pose.position.x = waypoint[0]
        target_marker.pose.position.y = waypoint[1]
        target_marker.pose.position.z = 0.0
        # marker.lifetime = rclpy.duration.Duration()  # Setting to zero so it never auto-deletes

        self.viz_target_pub.publish(target_marker)

    def visualize_ref_traj(self, ref_traj):
        marker_array = MarkerArray()
        for i in range(ref_traj.shape[1]):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.150
            marker.scale.y = 0.15
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = ref_traj[0, i]
            marker.pose.position.y = ref_traj[1, i]
            marker.pose.position.z = 0.0
            marker_array.markers.append(marker)
        print("ref_traj: ", ref_traj)
        self.viz_ref_traj_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    mpc_node = MPC()
    rclpy.spin(mpc_node)

    mpc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
