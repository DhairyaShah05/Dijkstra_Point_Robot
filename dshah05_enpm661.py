import numpy as np
import matplotlib.pyplot as plt 

class Node:
    
    # Initialize Data
	def __init__(self, x, y, cost, parent_node):

		self.x = x
		self.y = y
		self.cost = cost
		self.parent_node = parent_node
	
    # To support Node Comparision
	def __lt__(self,other):
		return self.cost < other.cost

def move_up(x,y,cost):
	y += 1
	cost += 1
	return x,y,cost

def move_down(x,y,cost):
	y -= 1
	cost += 1
	return x,y,cost

def move_left(x,y,cost):
	x -= 1
	cost += 1
	return x,y,cost

def move_right(x,y,cost):
	x += 1
	cost += 1
	return x,y,cost

def move_upright(x,y,cost):
	x += 1
	y += 1
	cost += np.sqrt(2)
	return x,y,cost

def move_downright(x,y,cost):
	x += 1
	y -= 1
	cost += np.sqrt(2)
	return x,y,cost

def move_upleft(x,y,cost):
	x -= 1
	y += 1
	cost += np.sqrt(2)
	return x,y,cost

def move_downleft(x,y,cost):
	x -= 1
	y -= 1
	cost += np.sqrt(2)
	return x,y,cost

def move_node(move,x,y,cost):
    if move == 'Up': 
        return move_up(x,y,cost)
    elif move == 'UpRight':   
        return move_upright(x,y,cost)
    elif move == 'Right':   
        return move_right(x,y,cost)
    elif move == 'DownRight':   
        return move_downright(x,y,cost)
    elif move == 'Down':   
        return move_down(x,y,cost)
    elif move == 'DownLeft':    
        return move_downleft(x,y,cost)
    elif move == 'Left':    
        return move_left(x,y,cost)
    elif move == 'UpLeft':    # Move upleft
        return move_upleft(x,y,cost)
    else:
        return None

def check_goal(current, goal): 
    
	if (current.x == goal.x) and (current.y == goal.y):
		return True
	else:
		return False
