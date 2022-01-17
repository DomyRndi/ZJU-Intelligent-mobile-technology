import numpy as np
import math

FLT_MAX = 100000.0


class Config(object):
    max_speed = 1200
    min_speed = -300
    max_yawrate = np.radians(100.0)  # max角速度
    max_accel = 3000  # max加速度
    max_dyawrate = 5  # max角加速度
    velocity_resolution = 100  # 分度
    yawrate_resolution = np.radians(5.0)  # 分度
    dt = 0.02  # 时间间隔
    predict_time = 0.2
    heading = 1
    clearance = 0.5
    velocity = 1


class DWA(object):
    def __init__(self):
        self.barriers = []  # 障碍物

        # Planner Settings
        self.vel = (0.0, 0.0)  # 速度 vx vw
        self.pose = (0.0, 0.0, 0.0)  # 位姿 x y celta
        self.goal = None  # 目标
        # 一些参数 需要设置 具体是多少还不清楚
        self.config = Config()

    def calculate_velocity_cost(self, velocity, config):
        return config.max_speed - velocity[0]

    def calculate_heading_cost(self, pose, goal):
        dx = goal[0] - pose[0]
        dy = goal[1] - pose[1]
        distance = math.sqrt(dx*dx+dy*dy)
        return distance
        # angle_error = math.atan2(dy, dx)
        # angle = pose[2]-angle_error
        # return math.fabs(math.atan2(math.sin(angle), math.cos(angle)))
        # return math.fabs(angle)

    def calculate_clearance_cost(self, pose, velocity, barriers, config):
        time = 0.0
        min = FLT_MAX
        pPose = pose
        while time < config.predict_time:
            pPose = self.motion(pPose, velocity, config.dt)
            for i in range(int(barriers.size / 2) - 1):
                dx = pPose[0] - barriers[i][0]
                dy = pPose[1] - barriers[i][1]
                r = math.sqrt(dx * dx + dy * dy)
                if r <= 100:
                    return FLT_MAX
                if r < min:
                    min = r
                i += 1
            time += config.dt
        return 100000.0 / math.pow(min / 100.0, 5)

    def motion(self, pose, velocity, dt):
        new_yaw = pose[2] + velocity[1] * dt
        new_x = pose[0] + velocity[0] * math.cos(new_yaw) * dt
        new_y = pose[1] + velocity[0] * math.sin(new_yaw) * dt
        new_pose = (new_x, new_y, new_yaw)
        return new_pose

    def planning(self, pose, velocity, goal, barriers, config):
        min_velocity = max(config.min_speed, velocity[0] - config.max_accel * config.dt)
        max_velocity = min(config.max_speed, velocity[0] + config.max_accel * config.dt)
        min_w = max(-config.max_yawrate, velocity[1] - config.max_dyawrate * config.dt)
        max_w = min(config.max_yawrate, velocity[1] + config.max_dyawrate * config.dt)

        nPossiblevelocity = (max_velocity - min_velocity) / config.velocity_resolution
        nPossiblew = (max_w - min_w) / config.yawrate_resolution
        possible_velocity = []
        possible_w = []

        for i in range(int(nPossiblevelocity)+1):
            possible_velocity.append(min_velocity + i * config.velocity_resolution)
            i += 1

        for j in range(int(nPossiblew)+1):
            possible_w.append(min_w + j * config.yawrate_resolution)
            j += 1

        best_velocity = velocity
        Cost = FLT_MAX
        for i in range(int(nPossiblevelocity)+1):
            for j in range(int(nPossiblew)+1):
                pPose = pose
                pVelocity = (possible_velocity[i], possible_w[j])
                pPose = self.motion(pPose, pVelocity, config.predict_time)
                cost_vel = self.calculate_velocity_cost(pVelocity, config)
                cost_head = self.calculate_heading_cost(pPose, goal)
                cost_clearance = self.calculate_clearance_cost(pose, pVelocity, barriers, config)
                cost = config.velocity * cost_vel + config.heading * cost_head
                       # + config.clearance * cost_clearance
                # print("cost", cost)
                if cost < Cost:
                    Cost = cost
                    best_velocity = pVelocity
                j += 1
            i += 1
        return best_velocity
