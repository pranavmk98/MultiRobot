'''
	@author: Richard Xu, Pranav Kumar, Professor Qin Chen
	@organization: Stanford Pre-Collegiate Studies 2016
	@task: Multiple Robot Grid Navigation
	This is a program to control multiple robots across a grid space so that they don't collide
	with obstacles and other hamsters. The program is exponential time with the number of hamsters,
	so any navigation across large space with >4 robots, or small space with >6 robots will be
	impractical.
'''

import sys 
import time
import Queue
import Tkinter as tk
from threading import Thread
from HamsterAPI.comm_ble import RobotComm 

def hecks(num):
	s = hex(num)[2:]
	if len(s) < 2:
		return "0" + s
	return s

#Generates the rgb string given r, g, b, in integer.
def rgb(c):
	return "#" + hecks(c[0]) + hecks(c[1]) + hecks(c[2])

frame = tk.Tk()
frame.wm_title("Multi-Robot Navigation")
COLORS = ((240, 255, 240), (255, 245, 238), (255, 160, 132), (50, 205 ,50), (238, 221, 130), (147, 112, 219), (255, 255, 255))
NODE_DIST = 60
NODE_SIZE = 20

'''
	Robot drive section - This will make them move based on the instructions.
	To accomplish this for multiple robot, we first wait for the robots to load using a 
	time.sleep() statement and the tkinter frame. Then, for each move, we create a
	thread for each robot to guide it to its new direction. The robots will move to its next
	location based on the instructions, and wait for others to finish using the t.join()
	statement. The only bug for the drive is that for robots in older version/low battery, it
	turns slower than new robots with full battery, which is what the right, left, and uturn
	are modeled over. As a result, they will not be able to turn enough, and as a result move
	away from the path.
'''
def move(robot, l, r, t):
	robot.set_wheel(0, l);
	robot.set_wheel(1, r);
	time.sleep(t);
	robot.set_wheel(0, 0);
	robot.set_wheel(1, 0);

def right(robot):
	move(robot, 60, 0, 0.8)

def left(robot):
	move(robot, 0, 60, 0.85)

def uturn(robot):
	move(robot, 30, -30, 1.7)

def forward(robot):
	move(robot, 30, 30 , 1)

def pid(robot):
	global interval;
	l = robot.get_floor(0);
	r = robot.get_floor(1);
	d = r - l;
	# print("l = %d, r = %d, d = %d" % (l, r, d));
	change = max(min(int(0.2 * d), 69), -69);
	robot.set_wheel(0, 30 - change);
	robot.set_wheel(1, 30 + change);
	time.sleep(1.0 / 20);
	return;

def indivDrive(instruction, robot, robotNum, stepNum):
	if instruction == 'F':
		forward(robot)
	elif instruction == 'L':
		left(robot)
	elif instruction == 'R':
		right(robot)
	elif instruction == 'U':
		uturn(robot)
	else:
		robot.set_musical_note(45)
		time.sleep(0.5)
		robot.set_musical_note(0)
		update(robotNum, stepNum + 1)
		return
	while not (robot.get_floor(0) < 30 and robot.get_floor(1) < 30):
		pid(robot)
	robot.set_wheel(0, 0)
	robot.set_wheel(1, 0)
	update(robotNum, stepNum + 1)
	robot.set_musical_note(45)
	time.sleep(0.5)
	robot.set_musical_note(0)
	# print "done"


def drive(instructions):
	comm = RobotComm(numRobot)
	comm.start()
	gRobotList = comm.robotList
	# return
	while (len(gRobotList) < numRobot):
		print "Waiting for robots"
		time.sleep(1)
	print "Robots on board. Driving start!"
	time.sleep(3)
	index = 0
	# for i in range(numRobot):
	# 	update(numRobot, 0)
	while index < len(instructions[0]):
		print "step #%d" % (index + 1)
		threads = []
		for i in range(numRobot):
			t = Thread(name="Moving robot %d" % (i + 1), target=indivDrive, args=(instructions[i][index], gRobotList[i], i, index))
			t.start()
			# time.sleep(3)
			threads.append(t)
		for t in threads:
			# print "Wait"
			t.join()
		# print "I'm done!"
		index += 1
		time.sleep(1)

'''
	GUI section. Draws the vertices on the screen with color.
'''

def draw_node(canvas, node, n_color):
	x = node[1][0]
	y = node[1][1]
	dist = NODE_DIST
	size = NODE_SIZE
	canvas.create_oval(x * dist - size, y * dist - size, x * dist + size, y * dist + size, fill=n_color)
	canvas.create_text(x * dist, y * dist, fill="black", text=node[0])
	return

def draw_background(canvas, node, n_color):
	x = node[1][0]
	y = node[1][1]
	dist = NODE_DIST
	size = NODE_SIZE + 5
	bg = canvas.create_oval(x * dist - size, y * dist - size, x * dist + size, y * dist + size, fill=n_color)
	canvas.tag_lower(bg)
	return

def draw_edge(canvas, node1, node2, e_color):
	x1 = node1[1][0]
	y1 = node1[1][1]
	x2 = node2[1][0]
	y2 = node2[1][1]
	dist = NODE_DIST
	edge = canvas.create_line(x1 * dist, y1 * dist, x2 * dist, y2 * dist, fill=e_color)
	canvas.tag_lower(edge)
	return

def draw_edge_special(canvas, node1, node2):
	x1 = int(node1[0]) + 1
	y1 = int(node1[1]) + 1
	x2 = int(node2[0]) + 1
	y2 = int(node2[1]) + 1
	dist = NODE_DIST
	edge = canvas.create_line(x1 * dist, y1 * dist, x2 * dist, y2 * dist, fill="forest green")
	return

def update(robotNum, stepNum):
	global path, COLORS,terminals
	color = rgb((int(COLORS[robotNum][0] * (len(path) - stepNum) / len(path)), int(COLORS[robotNum][1] * (len(path) - stepNum) / len(path)), int(COLORS[robotNum][2] * (len(path) - stepNum) / len(path))))
	x = int(path[stepNum][robotNum][0]) + 1
	y = int(path[stepNum][robotNum][1]) + 1
	dist = NODE_DIST
	size = NODE_SIZE
	terminals[robotNum].create_oval(x * dist - size, y * dist - size, x * dist + size, y * dist + size, fill=color)
	terminals[robotNum].create_text(x * dist, y * dist, fill="black", text=path[stepNum][robotNum])
	return

'''
	Main algorithm section. Below is an explanation of how the algorithm works:
	We use a tightly-coupled approach to solve the problem. While it is slower than decoupled
	algorithm, it is guaranteed to solve the problem optimally as long as a solution exists.
	
	Let n = number of robots, l = length of the grid, w = width of the grid.
	We first generate all possible tuples (x_1, y_1, x_2, y_2, ..., x_n, y_n) for the robots'
	location using recursion. Then, we connect them based on whether or not we can get to the
	other tuple with one move. The connection scheme goes as follows:
	For robot i at (x_i, y_i), it can move to
	#1. (x_i+1, y_i)
	#2. (x_i-1, y_i)
	#3. (x_i, y_i+1)
	#4. (x_i, y_i-1)
	#5. (x_i, y_i)
	However, if the new location will be occupied by robot 1..i-1 or an obstacle, or if 
	the new location is occupied by robot 1..i-1 right now, which will occupy (x_i, y_i)
	(to prevent head-on collision), or if the new coordinates are out of bounds, it is not
	valid.
	Once we generate the connections, we perform a breath-first search and keep track of how
	we get to each vertex from the starting one. Once we find the shortest path we convert it
	into direction based on the following scheme:
	We look at the robot's starting direction for each move (direction), and the direction it
	needs to get to the next location (newDir). Then, we see how much it needs to turn by to
	get to the new location. L = turn left, U = u-turn, R = turn right, F = go forward, S = stop.
'''
def indevToInstruction(path, j):
	# Assume that robot starts facing east.
	# 0 = E, 1 = N, 2 = W, 3 = S
	global start_dir
	direction = start_dir[j]
	instructions = []
	# print str(path)
	for i in range(len(path) - 1):
		newDir = None
		if int(path[i + 1][j][0]) - int(path[i][j][0]) == 1:  # East
			newDir = 0
		elif int(path[i + 1][j][1]) - int(path[i][j][1]) == -1:  # North
			newDir = 1
		elif int(path[i + 1][j][0]) - int(path[i][j][0]) == -1:  # West
			newDir = 2
		elif int(path[i + 1][j][1]) - int(path[i][j][1]) == 1:  # South
			newDir = 3
		else:
			newDir = -1
		# print("i = %d, direction = %d, newDir = %d" % (i, direction, newDir))
		if not newDir == -1:
			change = (newDir - direction + 4) % 4
			if change == 1:
				instructions.append('L')
			elif change == 2:
				instructions.append('U')
			elif change == 3:
				instructions.append('R')
			else:
				instructions.append('F')
			direction = newDir
		else:
			instructions.append('S')
	return instructions

path = None
def toInstruction(paths):
	#Also, generate the color for the paths.
	global path
	path = paths
	print "Path: \n" + str(paths)
	global terminals
	for i in range(len(terminals)):
		for j in range(len(paths) - 1):
			draw_edge_special(terminals[i], paths[j][i], paths[j+1][i])
	instructions = []
	for i in range(numRobot):
		instructions.append(indevToInstruction(paths, i))
	return instructions

def st(i, j):
	return str(i) + str(j)

def generateEdges(newLocSoFar, original, graph):
	global obstacles, l, w
	if len(newLocSoFar) < len(original):
		i = len(newLocSoFar)
		x_i = int(original[i][0])
		y_i = int(original[i][1])
		posLoc = [st(x_i, y_i)]
		if x_i < l - 1:
			posLoc.append(st(x_i + 1, y_i))
		if x_i > 0:
			posLoc.append(st(x_i - 1, y_i))
		if y_i < w - 1:
			posLoc.append(st(x_i, y_i + 1))
		if y_i > 0:
			posLoc.append(st(x_i, y_i - 1))
		for newLoc in posLoc:
			if not (newLoc in newLocSoFar or newLoc in obstacles):
				headOn = False;
				if newLoc in original:
					j = original.index(newLoc)
					if j < i and original[i] == newLocSoFar[j]:
						headOn = True
				if not headOn:
					newLocSoFar.append(newLoc)
					generateEdges(newLocSoFar, original, graph)
					del newLocSoFar[-1]
	else:
		graph[original].append(tuple(newLocSoFar))

numGenerated = 0
def generateVertices(soFar, graph):
	global numRobot, obstacles, l, w, numGenerated
	if len(soFar) < numRobot:
		for i in range(l):
			for j in range(w):
				loc = st(i, j)
				if (not loc in soFar) and (not loc in obstacles):
					soFar.append(loc)
					generateVertices(soFar, graph)
					del soFar[-1]
	else:
		numGenerated += 1
		if numGenerated % 1000 == 0:
			print "%d vertices generated." % numGenerated
		vertex = tuple(soFar)
		graph[vertex] = []
		generateEdges([], vertex, graph)

def bfs(start_nodes, graph):
	global end_nodes, mdgraph
	q = Queue.Queue()
	visited = dict()
	for vertex in graph:
		visited[vertex] = False
	q.put((start_nodes, [start_nodes]))
	while not q.empty():
		node = q.get()
		if not visited[node[0]]:
			visited[node[0]] = True
			# print("Visiting node " + str(node[0]))
			if node[0] == end_nodes:
				return toInstruction(node[1])
			for newNode in graph[node[0]]:
				if not visited[newNode]:
					q.put((newNode, node[1] + [newNode]))

'''
	The main control for the program. This has to be a thread because frame.mainloop() and
	this has to run at the same time.
'''
def main():
	global start_nodes, end_nodes, COLORS, nodes_location, canvas, l, terminals

	# Print the nodes
	canvas.create_text(l * 35, 10, text = "Main terminal")
	for node in nodes_location:
		if node[0] in start_nodes:
			draw_node(canvas, node, rgb(COLORS[start_nodes.index(node[0])]))
		else:
			draw_node(canvas, node, 'RoyalBlue1')
		if node[0] in end_nodes:
			draw_background(canvas, node, rgb(COLORS[end_nodes.index(node[0])]))
		# get list of names of connected nodes
		connected_nodes = graph[node[0]]
		# find location for each connected node and draw edge
		if connected_nodes:
			for connected_node in connected_nodes:
				# step into node locations
				for a_node in nodes_location:
					if connected_node == a_node[0]:
						draw_edge(canvas, node, a_node, 'RoyalBlue1')
	# Generate the terminals
	for i in range(numRobot):
		terminals[i].create_text(l * 35, 10, text = "Terminal #%d" % (i+1))
		for node in nodes_location:
			if node[0] == start_nodes[i]:
				draw_node(terminals[i], node, rgb(COLORS[i]))
			else:
				draw_node(terminals[i], node, 'RoyalBlue1')
			if node[0] == end_nodes[i]:
				draw_background(terminals[i], node, rgb(COLORS[i]))
			# get list of names of connected nodes
			connected_nodes = graph[node[0]]
			# find location for each connected node and draw edge
			if connected_nodes:
				for connected_node in connected_nodes:
					# step into node locations
					for a_node in nodes_location:
						if connected_node == a_node[0]:
							draw_edge(terminals[i], node, a_node, 'RoyalBlue1')
	# Generate multi-dimensional graph
	mdgraph = dict()
	print "Generating vertices"
	generateVertices([], mdgraph)
	# print str(mdgraph)
	print "Breath First Searching"
	instructions = bfs(start_nodes, mdgraph)
	print str(instructions)
	time.sleep(1)
	drive(instructions)
	print "Finished!"
	time.sleep(3)
	sys.exit(0)


'''
	The section that reads the input for the entire program.
'''
l = int(raw_input("Input length: ") or 4)
w = int(raw_input("Input width: ") or 4)
canvas = tk.Canvas(frame, bg="snow", width=l * 70, height=w * 70)
canvas.grid(row = 0, column = 0)
n = int(raw_input("Input #(obstacles): ") or 0)
obstacles = []
for i in range(n):
	obstacles.append(str(raw_input("Enter coordinates for obstacle #%d: " % (i+1))))
graph = dict()
nodes_location = []

for i in range(l):
	for j in range(w):
		name = str(i) + str(j)
		if not name in obstacles:
			connected = set();
			if i > 0 and not (st(i - 1, j)) in obstacles:
				connected.add(st(i - 1, j))
			if i < l - 1  and not (st(i + 1, j)) in obstacles:
				connected.add(st(i + 1, j))
			if j > 0  and not (st(i, j - 1)) in obstacles:
				connected.add(st(i, j - 1))
			if j < w - 1 and not (st(i, j + 1)) in obstacles:
				connected.add(st(i, j + 1))
			graph[name] = connected
			nodes_location.append((name, [i + 1, j + 1]))
numRobot = int(raw_input("Input #(robots): ") or 0)
terminals = []
for i in range(numRobot):
	terminals.append(tk.Canvas(frame, bg="snow", width=l * 70, height=w * 70))
	terminals[i].grid(row = (i + 1) / 4, column = (i + 1) % 4)
start_nodes = []
end_nodes = []
start_dir = []
for i in range(numRobot):
	start_nodes.append(raw_input("Input start coordinate for robot #%d: " % (i + 1)).strip())
	end_nodes.append(raw_input("Input end coordinate for robot #%d: " % (i + 1)).strip())
	start_dir.append(int(raw_input("Input the start direction for robot #%d (0 = E, 1 = N, ...): " % (i + 1)) or 0))
start_nodes = tuple(start_nodes)
end_nodes = tuple(end_nodes)
mainThread = Thread(target=main)
mainThread.start()
frame.mainloop()
