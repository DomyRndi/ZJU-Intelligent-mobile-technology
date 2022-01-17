import random
import math
import socket
import sys
import pygame
import time
import numpy as np
from vision import Vision
from action import Action
from debug import Debugger
from dwa import DWA
from zss_debug_pb2 import Debug_Msgs, Debug_Msg, Debug_Arc
show_animation = True

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList,
                 randArea, expandDis=100.0, goalSampleRate=20, maxIter=600):

        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.Xrand = randArea[0]
        self.Yrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
        """
        Pathplanning
        animation: flag for animation on or off
        """
        self.nodeList = {0: self.start}
        i = 0

        while True:

            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(rnd)  # get nearest node index to random point
            newNode = self.steer(rnd, nind)  # generate new node from that nearest node in direction of random point

            if self.__CollisionCheck(newNode, self.obstacleList):  # if it does not collide

                nearinds = self.find_near_nodes(newNode, 5)  # find nearest nodes to newNode
                newNode = self.choose_parent(newNode,
                                             nearinds)  # from that nearest nodes find the best parent to newNode
                self.nodeList[i + 100] = newNode  # add newNode to nodeList
                self.rewire(i + 100, newNode, nearinds)  # make newNode a parent of another node if necessary
                self.nodeList[newNode.parent].children.add(i + 100)

                if len(self.nodeList) > self.maxIter:
                    leaves = [key for key, node in self.nodeList.items() if
                              len(node.children) == 0 and len(self.nodeList[node.parent].children) > 1]
                    if len(leaves) > 1:
                        ind = leaves[random.randint(0, len(leaves) - 1)]
                        self.nodeList[self.nodeList[ind].parent].children.discard(ind)
                        self.nodeList.pop(ind)
                    else:
                        leaves = [key for key, node in self.nodeList.items() if len(node.children) == 0]
                        ind = leaves[random.randint(0, len(leaves) - 1)]
                        self.nodeList[self.nodeList[ind].parent].children.discard(ind)
                        self.nodeList.pop(ind)

            i += 1

            if animation and i % 5 == 0:
                self.Draw_graph(rnd)
            if i == 500:
                break
        lastIndex = self.get_best_last_index()
        return(self.gen_final_course(lastIndex))

    def path_validation(self):
        lastIndex = self.get_best_last_index()
        if lastIndex is not None:
            while self.nodeList[lastIndex].parent is not None:
                nodeInd = lastIndex
                lastIndex = self.nodeList[lastIndex].parent

                dx = self.nodeList[nodeInd].x - self.nodeList[lastIndex].x
                dy = self.nodeList[nodeInd].y - self.nodeList[lastIndex].y
                d = math.sqrt(dx ** 2 + dy ** 2)
                theta = math.atan2(dy, dx)
                if not self.check_collision_extend(self.nodeList[lastIndex].x, self.nodeList[lastIndex].y, theta, d):
                    self.nodeList[lastIndex].children.discard(nodeInd)
                    self.remove_branch(nodeInd)

    def remove_branch(self, nodeInd):
        for ix in self.nodeList[nodeInd].children:
            self.remove_branch(ix)
        self.nodeList.pop(nodeInd)

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.nodeList[i].x
            dy = newNode.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i].x, self.nodeList[i].y, theta, d):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode.cost = mincost
        newNode.parent = minind
        return newNode

    def steer(self, rnd, nind):
        # expand tree
        nearestNode = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = Node(nearestNode.x, nearestNode.y)
        newNode.x += self.expandDis * math.cos(theta)
        newNode.y += self.expandDis * math.sin(theta)
        newNode.cost = nearestNode.cost + self.expandDis
        newNode.parent = nind
        return newNode

    def get_random_point(self):
        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(0, self.Xrand), random.uniform(0, self.Yrand)]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y]
        return rnd

    def get_best_last_index(self):
        disglist = [(key, self.calc_dist_to_goal(node.x, node.y)) for key, node in self.nodeList.items()]
        goalinds = [key for key, distance in disglist if distance <= self.expandDis]
        if len(goalinds) == 0:
            return None
        mincost = min([self.nodeList[key].cost for key in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i
        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode, value):
        r = self.expandDis * value
        dlist = np.subtract(np.array([(node.x, node.y) for node in self.nodeList.values()]),
                            (newNode.x, newNode.y)) ** 2
        dlist = np.sum(dlist, axis=1)
        nearinds = np.where(dlist <= r ** 2)
        nearinds = np.array(list(self.nodeList.keys()))[nearinds]

        return nearinds

    def rewire(self, newNodeInd, newNode, nearinds):
        for i in nearinds:
            nearNode = self.nodeList[i]
            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)
            scost = newNode.cost + d
            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode.x, nearNode.y, theta, d):
                    self.nodeList[nearNode.parent].children.discard(i)
                    nearNode.parent = newNodeInd
                    nearNode.cost = scost
                    newNode.children.add(i)

    def check_collision_extend(self, nix, niy, theta, d):
        tmpNode = Node(nix, niy)
        for i in range(int(d / 5)):
            tmpNode.x += 5 * math.cos(theta)
            tmpNode.y += 5 * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False
        return True

    def Draw_graph(self, rnd=None):
        """
        Draw Graph
        """

        debugger = Debugger()
        package = Debug_Msgs()
        for node in self.nodeList.values():
            if node.parent is not None:
                debugger.draw_line(package, x1=self.nodeList[node.parent].x * 2 - 4500, y1=self.nodeList[node.parent].y * 2 - 3000, x2=node.x * 2 - 4500, y2=node.y * 2 - 3000)
        for node in self.nodeList.values():
            if len(node.children) == 0:
                debugger.draw_circle_small(package, x=int(node.x) * 2 - 4500, y=int(node.y) * 2 - 3000)
        debugger.draw_circle_small(package, x=self.start.x * 2 - 4500, y=self.start.y * 2 - 3000)
        debugger.draw_circle_small(package, x=self.end.x * 2 - 4500, y=self.end.y * 2 - 3000)
        lastIndex = self.get_best_last_index()
        if lastIndex is not None:
            path = self.gen_final_course(lastIndex)

            ind = len(path)
            while ind > 1:
                #pygame.draw.line(screen, (255, 0, 0), path[ind - 2], path[ind - 1])
                debugger.draw_line_green(package, x1=path[ind - 2][0] * 2 - 4500, y1=path[ind - 2][1] * 2 - 3000, x2=path[ind - 1][0] * 2 - 4500, y2=path[ind - 1][1] * 2 - 3000)
                print("finnal_line", path[ind - 2][0] * 2 - 4500, path[ind - 2][1] * 2 - 3000, path[ind - 1][0] * 2 - 4500, path[ind - 1][1] * 2 - 3000)
                ind -= 1
        debugger.send(package)



    def GetNearestListIndex(self, rnd):
        dlist = np.subtract(np.array([(node.x, node.y) for node in self.nodeList.values()]), (rnd[0], rnd[1])) ** 2
        dlist = np.sum(dlist, axis=1)
        minind = list(self.nodeList.keys())[np.argmin(dlist)]
        return minind

    def __CollisionCheck(self, node, obstacleList):

        for (sx, sy, ex, ey) in obstacleList:
            sx, sy, ex, ey = sx + 2, sy + 2, ex + 2, ey + 2
            if node.x > sx and node.x < sx + ex:
                if node.y > sy and node.y < sy + ey:
                    return False

        return True  # safe

def move_car(vision_car, x0, y0, x1, y1):
    action = Action()
    while True:
        self_x, self_y = vision_car.my_robot.x, vision_car.my_robot.y

        Fx = x1 - self_x
        Fy = y1 - self_y

        theta = vision_car.my_robot.orientation
        theta_circle = theta  # 保留信息
        theta = 2 * math.pi + theta if theta < 0 else theta

        alpha = math.atan2(Fy, Fx)
        alpha_circle = alpha
        alpha = 2 * math.pi + alpha if alpha < 0 else alpha

        error = abs(theta - alpha)
        error_circle = theta_circle - alpha_circle  # 求车头方向和指向目标方向的夹角（非绝对值）
        error = 2 * math.pi - error if error > math.pi else error
        if error_circle >= math.pi*10/180 and error_circle <= math.pi:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
            action.sendCommand(vx=0, vw=-1)
        # else:
        #     action.sendCommand(vx=500, vw=1)


        if error_circle >= 0 and error_circle <= math.pi*10/180:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
            action.sendCommand(vx=10, vw=-1)
        # else:
        #     action.sendCommand(vx=500, vw=1)
        if error_circle >= math.pi and error_circle <= math.pi*350/180:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
            action.sendCommand(vx=0, vw=3)
        if error_circle >= math.pi*350/180 and error_circle <= math.pi*2:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
            action.sendCommand(vx=10, vw=1)
        if error_circle >= -math.pi and error_circle <= -math.pi * 10 / 180:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
            action.sendCommand(vx=0, vw=3)

        if error_circle >= -math.pi * 10 / 180 and error_circle <= 0:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
            action.sendCommand(vx=10, vw=1)
        if error_circle >= -math.pi*350/180 and error_circle <= -math.pi:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
            action.sendCommand(vx=0, vw=-3)
        if error_circle >= -math.pi*2 and error_circle <= -math.pi*350/180:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
            action.sendCommand(vx=10, vw=-1)

        while error > (3 / 180) * math.pi:  # 给定一个停止旋转范围
            theta = vision_car.my_robot.orientation
            theta = 2 * math.pi + theta if theta < 0 else theta
            alpha = math.atan2(Fy, Fx)
            alpha = 2 * math.pi + alpha if alpha < 0 else alpha
            error = abs(theta - alpha)
            error = 2 * math.pi - error if error > math.pi else error

        action.sendCommand(vx=300, vw=0)  # 非旋转时刻前进
        if abs(self_x - x1)<=30 and abs(self_y - y1)<=30:
            action.sendCommand(vx=0, vw=0)
            break
    print('down')  # 阶段测试


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None
        self.children = set()


def main():
    print("start RRT path planning")
    vision = Vision()
    action = Action()
    debugger = Debugger()
    package = Debug_Msgs()
    time.sleep(1.0)
    # ====Search Path with RRT====

    # 1. path planning & velocity planning
    self_x, self_y = (vision.my_robot.x + 4500) / 2, (vision.my_robot.y + 3000) / 2
    yellow_robot = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    for i in range(0, 16):
        yellow_robot[i] = vision.yellow_robot[i]
    goal_x, goal_y = (-2500 + 4500) / 2, (-2000 + 3000) / 2  # 目标点
    obstacleList = []
    for i in range(0, 16):
        obstacleList.append([(yellow_robot[i].x + 4500-200) / 2, (yellow_robot[i].y + 3000-200) / 2, 200, 200])  # [x,y,size]

    # Set Initial parameters
    XDIM, YDIM = 4500, 3000
    rrt = RRT(start=[self_x, self_y], goal=[goal_x, goal_y],
              randArea=[XDIM, YDIM], obstacleList=obstacleList)
    path = rrt.Planning(animation=show_animation)
    time.sleep(0.01)  # 延时
    path.reverse()
    i = 0
    while True:
        if path[i][0] != goal_x and path[i][1] != goal_y:
            i += 1
        else:
            break
    print("path number:", i)
    for path_i in range(0, i+1):
        path[path_i][0] = path[path_i][0]*2-4500
        path[path_i][1] = path[path_i][1]*2-3000
        path_i += 1
    reversed_path = []
    for j in reversed(path):
        reversed_path.append(j)
    path_2 = np.vstack((path, reversed_path))
    path_2 = np.vstack((path_2, path))
    path_2 = np.vstack((path_2, reversed_path))
    path_2 = np.vstack((path_2, path))
    path = path_2

    # dwa_planning
    dwa = DWA()
    for j in range(16):
        dwa.barriers.insert(j, [vision.yellow_robot[j].x, vision.yellow_robot[j].y])
    path_i = 1
    while path_i < 5*(i+1):
        dwa.pose = (vision.my_robot.x, vision.my_robot.y, vision.my_robot.orientation)
        dwa.goal = (path[path_i][0], path[path_i][1])
        dwa.vel = dwa.planning(dwa.pose, dwa.vel, dwa.goal,
                                np.array(dwa.barriers, np.float32), dwa.config)
        dwa.pose = dwa.motion(dwa.pose, dwa.vel, dwa.config.dt)
        if dwa.pose[0] <= dwa.goal[0] + 100 and dwa.pose[0] >= dwa.goal[0] - 100 and dwa.pose[1] <= dwa.goal[1] + 100 and dwa.pose[1] >= dwa.goal[1] - 100:
            path_i += 1
            # dx = path[path_i][0] - path[path_i - 1][0]
            # dy = path[path_i][1] - path[path_i - 1][1]
            # distance = math.sqrt(dx * dx + dy * dy)
            # if distance <= 500:
            #     dwa.vel = (0, dwa.vel[1])
            # else:
            #     dwa.vel = (dwa.vel[0] , dwa.vel[1])
            # action.sendCommand(vx=dwa.vel[0], vy=0, vw=dwa.vel[1])
            # time.sleep(0.05)
        Fy = dwa.goal[1]-vision.my_robot.y
        Fx = dwa.goal[0]-vision.my_robot.x
        theta = vision.my_robot.orientation
        alpha = math.atan2(Fy, Fx)
        error = theta - alpha
        angle = math.atan2(math.sin(error), math.cos(error))
        angle_abs = abs(angle)
        if angle >= 0:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
            action.sendCommand(vx=0, vw=-2)
        else:
            action.sendCommand(vx=0, vw=2)

        while angle_abs > (25 / 180) * math.pi:  # 给定一个停止旋转范围
            theta = vision.my_robot.orientation
            alpha = math.atan2(Fy, Fx)
            error = theta - alpha
            angle = math.atan2(math.sin(error), math.cos(error))
            angle_abs = abs(angle)

        # 2. send command
        action.sendCommand(vx=dwa.vel[0], vy=0, vw=dwa.vel[1])
    action.sendCommand(vx=0, vy=0, vw=0)


if __name__ == '__main__':
    main()
