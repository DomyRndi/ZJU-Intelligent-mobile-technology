from vision import Vision
from action import Action
# from debug import Debugger
# from prm import PRM
from dwa import DWA
import numpy as np
import time

if __name__ == '__main__':
    vision = Vision()
    action = Action()

    # debugger = Debugger()
    # planner = PRM()
    time.sleep(2)
    dwa = DWA()
    m = 0
    for m in range(16):
        dwa.barriers.insert(m, [vision.yellow_robot[m].x, vision.yellow_robot[m].y])
    print("barriers", dwa.barriers[m])
    # for i in range(15):
    # start_x, start_y = vision.my_robot.x, vision.my_robot.y
    goal_x, goal_y = -3300, -2100
    goal = [goal_x, goal_y]
    dwa.goal = goal
    #origin = [vision.my_robot.x, vision.my_robot.y]
    origin = [3300, 2100]
    n = 0
    while n < 5:
        #action.controlObs(vision)
        # action.controlObs(vision)
        j = 0
        for j in range(16):
            dwa.barriers[j] = [vision.yellow_robot[j].x, vision.yellow_robot[j].y]
        print("barriers", dwa.barriers[j])
        dwa.pose = (vision.my_robot.x, vision.my_robot.y, vision.my_robot.orientation)
        print("pose:", dwa.pose)

        dwa.vel = dwa.planning(dwa.pose, dwa.vel, dwa.goal, np.array(dwa.barriers, np.float32), dwa.config)
        print("velocity", dwa.vel)
        dwa.pose = dwa.motion(dwa.pose, dwa.vel, dwa.config.dt)
        if dwa.pose[0] <= dwa.goal[0] + 100 and dwa.pose[0] >= dwa.goal[0] - 100 and dwa.pose[1] <= dwa.goal[1] + 100 and dwa.pose[1] >= dwa.goal[1] - 100 and dwa.goal == goal:
            dwa.goal = origin
        if dwa.pose[0] <= dwa.goal[0] + 100 and dwa.pose[0] >= dwa.goal[0] - 100 and dwa.pose[1] <= dwa.goal[1] + 100 and dwa.pose[1] >= dwa.goal[1] - 100 and dwa.goal == origin:
            dwa.goal = goal
            n += 1
        action.sendCommand(vx=dwa.vel[0], vy=0, vw=dwa.vel[1])
        time.sleep(0.001)
    action.sendCommand(vx=0, vy=0, vw=0)
