from vision import Vision
from action import Action
from debug import Debugger
import time
import math

if __name__ == '__main__':
   vision = Vision()
   action = Action()
   debugger = Debugger()
   time.sleep(1.0)
   self_x, self_y = vision.my_robot.x, vision.my_robot.y
   goal_x, goal_y = -self_x, -self_y  # 目标点
   while True:
      # 1. path planning & velocity planning
      self_x, self_y = vision.my_robot.x, vision.my_robot.y
      yellow_robot = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
      for i in range(0, 15):
         yellow_robot[i] = vision.yellow_robot[i]
      if abs(self_x - goal_x)<=30 and abs(self_y - goal_y)<=30:
         self_x, self_y = vision.my_robot.x, vision.my_robot.y
         goal_x, goal_y = -self_x, -self_y  # 目标点
      p0 = 500
      da = 500
      Ka = 2
      Kr = 100000000000  #设置参数，现在这个参数还可以，但肯定不是最优的
      d =((self_x-goal_x)**2+(self_y-goal_y)**2)**0.5  #距离目标距离
      if d <= da:
         Fattx = -2 * Ka * (self_x - goal_x)
         Fatty = -2 * Ka * (self_y - goal_y)
      else:
         Fattx = -2 * Ka * da * (self_x - goal_x) / d
         Fatty = -2 * Ka * da * (self_y - goal_y) / d
      Fx = Fattx
      Fy = Fatty
      for i in range(0, 15):
         p = ((self_x - yellow_robot[i].x)**2+(self_y-yellow_robot[i].y)**2)**0.5
         #print(self_x, yellow_robot[i].x, self_y, yellow_robot[i].y)
         if p <= p0:
            Frepx = Kr * (1 / p - 1 / p0) * (1 / p ** 3) * (self_x - yellow_robot[i].x)
            Frepy = Kr * (1 / p - 1 / p0) * (1 / p ** 3) * (self_y - yellow_robot[i].y)
         else:
            Frepx = Frepy = 0
         Fx = Fx+Frepx  #求合力
         Fy = Fy+Frepy
      print(Fx,Fy)
      # Do something

      # 2. send command
      theta = vision.my_robot.orientation
      theta = 2 * math.pi + theta if theta < 0 else theta
      theta_circle = theta  # 保留信息
      alpha = math.atan2(Fy, Fx)
      alpha = 2 * math.pi + alpha if alpha < 0 else alpha
      alpha_circle=alpha
      error = abs(theta - alpha)
      error_circle = theta_circle - alpha_circle   #求车头方向和指向目标方向的夹角（非绝对值）
      error = 2 * math.pi - error if error > math.pi else error
      if error_circle >= 0 :   #判断夹角大小，大于零逆时针；小于零，顺时针转
         action.sendCommand(vx=0, vw=-2)
      else:
         action.sendCommand(vx=0, vw=2)

      while error > (15 / 180) * math.pi:  #给定一个停止旋转范围
         theta = vision.my_robot.orientation
         theta = 2 * math.pi + theta if theta < 0 else theta
         alpha = math.atan2(Fy, Fx)
         alpha = 2 * math.pi + alpha if alpha < 0 else alpha
         error = abs(theta - alpha)
         error = 2 * math.pi - error if error > math.pi else error

      action.sendCommand(vx=1000, vw=0)  #非旋转时刻前进
      print('down')  #阶段测试
      time.sleep(0.002)  #延时