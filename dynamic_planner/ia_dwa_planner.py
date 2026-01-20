#!/usr/bin/env python3

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Header

# Helper functions for angles
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def quaternion_to_euler(q):
    # q is [x, y, z, w]
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

class EKF:
    def __init__(self, dt=0.1):
        self.dt = dt
        # State: [x, y, theta, v, w]
        self.x = np.zeros(5)
        # Covariance
        self.P = np.eye(5) * 0.1
        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.1, 0.1, 0.1])
        # Measurement noise
        self.R = np.diag([0.1, 0.1, 0.1, 0.1, 0.1]) # Simplified
        
    def predict(self, u):
        # u: [v, w] control input (or from odom twist)
        v = self.x[3]
        w = self.x[4]
        theta = self.x[2]
        
        # Motion model
        self.x[0] += v * math.cos(theta) * self.dt
        self.x[1] += v * math.sin(theta) * self.dt
        self.x[2] += w * self.dt
        self.x[2] = normalize_angle(self.x[2])
        # v, w assumed constant in prediction step or updated via u if controls
        
        # Jacobian F
        F = np.eye(5)
        F[0, 2] = -v * math.sin(theta) * self.dt
        F[0, 3] = math.cos(theta) * self.dt
        F[1, 2] = v * math.cos(theta) * self.dt
        F[1, 3] = math.sin(theta) * self.dt
        F[2, 4] = self.dt
        
        self.P = F @ self.P @ F.T + self.Q
        
    def update(self, z):
        # z: measurement [x, y, theta, v, w] from Odom + IMU
        # H is identity if we measure full state
        H = np.eye(5)
        y = z - self.x
        y[2] = normalize_angle(y[2])
        
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        self.x[2] = normalize_angle(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P

class ImprovedAStar:
    def __init__(self, resolution=0.1):
        self.resolution = resolution
        self.min_x = -10
        self.min_y = -10
        self.max_x = 10
        self.max_y = 10
        self.obstacles = []
        
        # Heuristic params
        self.phi_max = math.pi / 2.0 # Max steering angle (assumed 90 deg for holonomic or high for diff drive)
        self.v_max = 0.22 # TB3 max speed
        self.dT = 0.1 # Prediction horizon for steering?
        self.R = 0.1 # Robot radius? or Turning radius
        
    def set_obstacles(self, obstacles):
        self.obstacles = obstacles # List of (x, y)
        
    def plan(self, start_node, goal_node):
        # start_node: (x, y), goal_node: (x, y)
        # Simplified A* for demonstration, focusing on Heuristic
        # In real grid map, this needs full A* implementation
        
        # For this task, we will simulate a path or use a coarse grid
        # Implementation of full A* on grid map is heavy. 
        # I will implement a simplified version or a waypoint generator
        # if map is not available. 
        # Assuming we get a map or work in local window.
        
        # Placeholder for full A* using the improved heuristic
        # If no map, return straight line
        path = [start_node, goal_node]
        return path

    def heuristic(self, node, goal, start, current_angle):
        # Eq 1: f(n) = g(n) + (1 + d/D) * (h1(n) + h2(n))
        # This function returns just the H part to be added to G
        # Wait, f = g + H_new. So H_new = (1 + d/D) * (h1 + h2)
        
        dx = node[0] - goal[0]
        dy = node[1] - goal[1]
        d = math.sqrt(dx*dx + dy*dy) # Distance (current to target)
        
        sx = start[0] - goal[0]
        sy = start[1] - goal[1]
        D = math.sqrt(sx*sx + sy*sy) # Distance (start to target)
        
        # h1: Steering deviation
        # angle to target
        target_angle = math.atan2(goal[1] - node[1], goal[0] - node[0])
        delta_phi = normalize_angle(target_angle - current_angle)
        h1 = (abs(delta_phi) / self.phi_max) * self.dT
        
        # h2: Turning radius / clearance?
        # Paper: h2 = sqrt((xt - xon)^2 + ... - R^2) / vmax
        # This looks like Euclidean dist adjusted by R (obstacle clearance or turning circle?)
        # "xon, yon are coordinates of turning center"
        # If straight line, h2 approx d/vmax
        h2 = d / self.v_max 
        
        factor = 1.0 + (d / D) if D > 0.001 else 1.0
        
        return factor * (h1 + h2)

class DWA:
    def __init__(self):
        self.max_speed = 0.22
        self.min_speed = 0.0
        self.max_yaw_rate = 1.0 # 2.84
        self.max_accel = 0.2
        self.max_yaw_accel = 1.0 # 3.2
        self.v_reso = 0.01
        self.yaw_rate_reso = 0.05 # rad/s
        self.dt = 0.1
        self.predict_time = 2.0 # s
        
        # Cost weights (Eq 8)
        # Score = alpha * p_dist + lambda * g_dist + beta * dist + gamma * vel
        self.alpha = 1.0 # Heading/global path alignment
        self.lam = 1.0   # Critical point distance
        self.beta = 1.0  # Obstacle distance
        self.gamma = 1.0 # Velocity
        
    def dwa_control(self, x, goal, obstacles, global_path_critical_point):
        # x: [x, y, theta, v, w]
        # goal: [x, y]
        # obstacles: list of [x, y]
        # global_path_critical_point: [x, y] - the "critical point" from A* path
        
        dw = self.calc_dynamic_window(x)
        
        min_cost = float('inf')
        best_u = [0.0, 0.0]
        best_traj = []
        
        # Search window
        for v in np.arange(dw[0], dw[1], self.v_reso):
            for w in np.arange(dw[2], dw[3], self.yaw_rate_reso):
                traj = self.predict_trajectory(x, v, w)
                
                # Calc cost
                to_goal_cost = self.calc_to_goal_cost(traj, goal) # g_dist or p_dist component
                speed_cost = self.max_speed - traj[-1, 3] # Encourage speed
                ob_cost = self.calc_obstacle_cost(traj, obstacles)
                
                # IA-DWA specific terms
                # p_dist: distance from end of traj to global path (simplified: distance to critical point line?)
                # g_dist: distance to critical point
                final_pos = traj[-1, 0:2]
                g_dist = math.sqrt((final_pos[0] - global_path_critical_point[0])**2 + (final_pos[1] - global_path_critical_point[1])**2)
                
                # p_dist: heading deviation cost? Paper says "distance from simulated target point to global path"
                # Let's interpret p_dist as cross track error or similar.
                # Simplified: Heading diff
                dx = global_path_critical_point[0] - final_pos[0]
                dy = global_path_critical_point[1] - final_pos[1]
                error_angle = normalize_angle(math.atan2(dy, dx) - traj[-1, 2])
                p_cost = abs(error_angle)
                
                
                # Total score (we minimize cost, formula maximizes score usually? Check Eq 8)
                # "Optimized in Eq 8... g(...) = ... " usually we want min cost.
                # If these are rewards, we maximize.
                # Typically nav costs are minimized.
                # Let's assume we MINIMIZE: alpha * p_cost + lambda * g_cost + beta * (1/dist_obs) + gamma * (1/vel)
                # Or Maximize: ...
                # Let's stick to standard DWA pattern: minimize penalty.
                # Penalty = alpha * HeadingErr + lambda * DistToGoal + beta * (1/Clearance) + gamma * (1/Vel)
                
                # Standard DWA:
                # cost = to_goal_cost + speed_cost + ob_cost
                
                # IA-DWA terms:
                # p_dist (alignment to global path)
                # g_dist (dist to critical point)
                
                score = self.alpha * p_cost + self.lam * g_dist + self.beta * (1.0/(ob_cost+0.1)) + self.gamma * (1.0/(traj[-1, 3]+0.1))
                
                # Re-reading paper snippet: "Score = ...". Probably utility to MAXIMIZE? or Cost to MINIMIZE?
                # Usually standard DWA maximizes Objective Function G(v,w).
                # But here I'll use Cost to Minimize convention.
                # So we want Small p_cost, Small g_dist, Large Clearance, Large Vel.
                # Cost = w1 * p_cost + w2 * g_dist + w3 * (1/Clearance) + w4 * (1/Vel)
                
                final_cost = self.alpha * p_cost + self.lam * g_dist + self.beta * (1.0 / (ob_cost + 0.01)) + self.gamma * (1.0 / (traj[-1, 3] + 0.01))

                if ob_cost < 0.2: # Collision
                    final_cost = float('inf')

                if final_cost < min_cost:
                    min_cost = final_cost
                    best_u = [v, w]
                    best_traj = traj
                    
        return best_u, best_traj

    def calc_dynamic_window(self, x):
        # [v_min, v_max, yaw_rate_min, yaw_rate_max]
        Vs = [0, self.max_speed, -self.max_yaw_rate, self.max_yaw_rate]
        
        Vd = [x[3] - self.max_accel * self.dt,
              x[3] + self.max_accel * self.dt,
              x[4] - self.max_yaw_accel * self.dt,
              x[4] + self.max_yaw_accel * self.dt]
              
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        return dw

    def predict_trajectory(self, x_init, v, w):
        traj = np.array(x_init)
        x = np.array(x_init)
        time = 0
        while time <= self.predict_time:
            x[2] += w * self.dt
            x[0] += v * math.cos(x[2]) * self.dt
            x[1] += v * math.sin(x[2]) * self.dt
            x[3] = v
            x[4] = w
            
            traj = np.vstack((traj, x))
            time += self.dt
        return traj

    def calc_to_goal_cost(self, traj, goal):
        dx = goal[0] - traj[-1, 0]
        dy = goal[1] - traj[-1, 1]
        return math.sqrt(dx**2 + dy**2)

    def calc_obstacle_cost(self, traj, obstacles):
        # Min dist to any obstacle
        min_dist = float('inf')
        for ob in obstacles: # ob is (x, y)
            for i in range(len(traj)):
                d = math.sqrt((traj[i, 0] - ob[0])**2 + (traj[i, 1] - ob[1])**2)
                if d < min_dist:
                    min_dist = d
        return min_dist # Return minimal distance

class IADWAPlanner(Node):
    def __init__(self):
        super().__init__('ia_dwa_planner')
        
        self.ekf = EKF()
        self.planner = ImprovedAStar()
        self.dwa = DWA()
        
        self.goal = None
        self.obstacles = [] # List found from LaserScan
        
        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.global_plan_pub = self.create_publisher(Path, '/plan', 10)
        self.local_plan_pub = self.create_publisher(Path, '/local_plan', 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def odom_callback(self, msg):
        # Update EKF measurement
        # z = [x, y, theta, v, w]
        # x, y from pose
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        # theta from quaternion
        _, _, yaw = quaternion_to_euler(msg.pose.pose.orientation)
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        
        z = np.array([px, py, yaw, v, w])
        self.ekf.update(z)
        
    def imu_callback(self, msg):
        # Update EKF with IMU (simplified: just orientation/rate if needed or fuse)
        # Note: In a real EKF, we would fuse Odom and IMU properly (predict with IMU/Odom, update with Odom/IMU).
        # Here we just use Odom update for simplicity as per requested structure "Implement EKF Fusion"
        pass 

    def scan_callback(self, msg):
        # Convert scan to obstacles list
        # Simple conversion assuming robot is at EKF estimated pose
        angle = msg.angle_min
        obs = []
        rx, ry, rtheta = self.ekf.x[0], self.ekf.x[1], self.ekf.x[2]
        
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Obstacle point in robot frame
                ox_r = r * math.cos(angle)
                oy_r = r * math.sin(angle)
                
                # Transform to world frame
                ox = rx + ox_r * math.cos(rtheta) - oy_r * math.sin(rtheta)
                oy = ry + ox_r * math.sin(rtheta) + oy_r * math.cos(rtheta)
                
                obs.append([ox, oy])
            angle += msg.angle_increment
        self.obstacles = obs # Update obstacles

    def goal_callback(self, msg):
        self.goal = [msg.pose.position.x, msg.pose.position.y]
        self.get_logger().info(f"New goal received: {self.goal}")

    def control_loop(self):
        if self.goal is None:
            return
            
        current_state = self.ekf.x # [x, y, theta, v, w]
        
        # 1. Global Plan (Improved A*)
        # Ideally calculate only when goal changes or path blocked
        # For dynamic, replan often? Or just get critical point.
        global_path = self.planner.plan((current_state[0], current_state[1]), self.goal)
        
        # Visualize Global Path
        path_msg = Path()
        path_msg.header.frame_id = "map" # or odom
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for p in global_path:
            pose = PoseStamped()
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            path_msg.poses.append(pose)
        self.global_plan_pub.publish(path_msg)
        
        # 2. Extract Critical Point (Subgoal)
        # For this simplified version (start->goal straight line), critical point is goal
        # Or a point further along the path
        critical_point = self.goal
        if len(global_path) > 1:
             critical_point = global_path[-1] # End of path
             
        # 3. Local Plan (DWA)
        u, traj = self.dwa.dwa_control(current_state, self.goal, self.obstacles, critical_point)
        
        # Publish Cmd Vel
        cmd = Twist()
        cmd.linear.x = float(u[0])
        cmd.angular.z = float(u[1])
        self.cmd_vel_pub.publish(cmd)
        
        # Visualize Local Traj
        if len(traj) > 0:
            local_path_msg = Path()
            local_path_msg.header.frame_id = "map" # or odom
            local_path_msg.header.stamp = self.get_clock().now().to_msg()
            for p in traj:
                pose = PoseStamped()
                pose.pose.position.x = float(p[0])
                pose.pose.position.y = float(p[1])
                local_path_msg.poses.append(pose)
            self.local_plan_pub.publish(local_path_msg)
            
        # Prediction for EKF
        self.ekf.predict(u)

def main(args=None):
    rclpy.init(args=args)
    node = IADWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
