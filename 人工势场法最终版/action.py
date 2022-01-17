import socket
import sys
import time
import random
import math

from zss_cmd_pb2 import Robots_Command, Robot_Command

class Action(object):
	def __init__(self):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.command_address = ('localhost', 50001)
		self.obs_address = ('localhost', 50002)
		self.obs_goal = [[-3200 + 375 * (i+1), 1000] for i in range(16)]

	def sendCommand(self, vx=0, vy=0, vw=0):
		commands = Robots_Command()
		command = commands.command.add()
		command.robot_id = 0
		command.velocity_x = vx
		command.velocity_y = vy
		command.velocity_r = vw
		command.kick = False
		command.power = 0
		command.dribbler_spin = False
		self.sock.sendto(commands.SerializeToString(), self.command_address)

	def controlObs(self, vision):
		commands = Robots_Command()
		for i in [3,4,6,9,12]:
			command = commands.command.add()
			command.robot_id = i
			# calculate goal
			dist2goal = math.sqrt((vision.yellow_robot[i].x-self.obs_goal[i][0])**2
				+(vision.yellow_robot[i].y-self.obs_goal[i][1])**2)
			if dist2goal < 200 or self.obs_goal[i][1] == 0:
				if i <= 3:
					self.obs_goal[i][1] = -1500 + random.random()*3000
				elif i < 8:
					self.obs_goal[i][1] = -3000 + random.random()*4000
				elif i < 12:
					self.obs_goal[i][1] = -1000 + random.random()*4000
				else:
					self.obs_goal[i][1] = -1500 + random.random()*3000
				# self.obs_goal[i][1] = -2000 + random.random()*4000
			# calculate speed
			angle2goal = math.atan2(self.obs_goal[i][1]-vision.yellow_robot[i].y,
				self.obs_goal[i][0]-vision.yellow_robot[i].x)
			command.velocity_x = 400 * math.cos(angle2goal-vision.yellow_robot[i].orientation)
			command.velocity_y = 400 * math.sin(angle2goal-vision.yellow_robot[i].orientation)
			command.velocity_r = 0
			command.kick = False
			command.power = 0
			command.dribbler_spin = False
		self.sock.sendto(commands.SerializeToString(), self.obs_address)

if __name__ == '__main__':
	action_module = Action()
	while True:
		action_module.sendCommand(vx=250, vy=5, vw=2)
		time.sleep(0.02)
