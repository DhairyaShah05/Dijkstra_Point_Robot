# Importing Necessary Libraries
import numpy as np
import matplotlib.pyplot as plt 
import time
import heapq 

# Creating a Class for Node with it's parameters as X,Y Coordinates, Cost at each Node, it's Parent
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

#Below functions will define the moves for up, down, left, right, upright, upleft, downright and downleft

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
      
# Function to move nodes according to the action set defined.

def move_node(move,x,y,cost):
    if move == 'Up':    # Move up
        return move_up(x,y,cost)
    elif move == 'UpRight':    # Move upright
        return move_upright(x,y,cost)
    elif move == 'Right':    # Move right
        return move_right(x,y,cost)
    elif move == 'DownRight':    # Move downright
        return move_downright(x,y,cost)
    elif move == 'Down':    # Move down
        return move_down(x,y,cost)
    elif move == 'DownLeft':    # Move downleft
        return move_downleft(x,y,cost)
    elif move == 'Left':    # Move left
        return move_left(x,y,cost)
    elif move == 'UpLeft':    # Move upleft
        return move_upleft(x,y,cost)
    else:
        return None


# Function to define an obsatcle space and buffer space.

def plot_map(width, height):
    
    # Generating Obstacle Space
    obstacle_space = np.full((height, width),0)
    
    for y in range(0, height) :
        for x in range(0, width):
            
            # Plotting Buffer Space for the Obstacles using Half Plane Equations
            
            # Rectangle 1 Obastacle
            r11_buffer = (x + 5) - 100  
            r12_buffer = (y + 5) - 100
            r13_buffer = (x - 5) - 175
            r14_buffer = (y - 5) - 500    # No need to define lower most line at boundry
            
            # Rectangle 2 Obastacle
            r21_buffer = (x + 5) - 275  
            r22_buffer = (y + 5) - 0  # No need to define lower most line at boundry
            r23_buffer = (x - 5) - 350
            r24_buffer = (y - 5) - 400 
            
            # Hexagon Obstacle
            h6_buffer = (y + 5) +  0.58*(x + 5) - 431.82
            h5_buffer = (y + 5) + 0.58*(x - 5) - 231.72
            h4_buffer = (x - 6.5) - 704.9
            h3_buffer = (y - 5) + 0.58*(x - 5) - 731.78
            h2_buffer = (y - 5) - 0.58*(x + 5) - 68.23
            h1_buffer = (x + 6.5) - 445.1
            
            # Block Obstacle
            t1_buffer = (x + 5) - 900
            t2_buffer = (x + 5) - 1020
            t3_buffer = (x - 5) - 1100
            t4_buffer = (y + 5) - 50
            t5_buffer = (y - 5) - 125
            t6_buffer = (y + 5) - 375
            t7_buffer = (y - 5) - 450
            
            # Setting the line constrain to obatain the obstacle space with buffer
            if((t1_buffer>0 and t2_buffer<0 and t4_buffer>0 and t5_buffer<0) or(t2_buffer>0 and t3_buffer<0 and t4_buffer>0 and t7_buffer<0) or (t6_buffer>0 and t7_buffer<0 and t1_buffer>0 and t2_buffer<0) or (r11_buffer>0 and r12_buffer>0 and r13_buffer<0 and r14_buffer<0) or (r21_buffer>0 and r23_buffer<0 and r24_buffer<0 and r22_buffer>0) or (h6_buffer<0 and h5_buffer>0 and h4_buffer>0 and h3_buffer>0 and h2_buffer>0 and h1_buffer<0)):
                obstacle_space[y, x] = 1
             
             
            # Plotting Actual Object Space Half Plane Equations
            
            # Wall Obstacles
            w1 = (y) - 5
            w2 = (y) - 495
            w3 = (x) - 5
            w4 = (x) - 1195 
            
            # Rectangle 1 Obastacle
            r11 = (x) - 100  
            r12 = (y) - 100
            r13 = (x) - 175
            r14 = (y) - 500
            
            # Rectangle 2 Obastacle
            r21 = (x) - 275  
            r22 = (y) - 0
            r24 = (x) - 350
            r23 = (y) - 400 
            
            # Hexagon Obstacle
            h6 = (y) +  0.58*(x) - 431.82
            h5 = (y) - 0.58*(x) + 231.72
            h4 = (x) - 704.9
            h3 = (y) + 0.58*(x) - 731.78
            h2 = (y) - 0.58*(x) - 68.23
            h1 = (x) - 445.1 
            
            # Block Obstacle
            t1 = (x) - 900
            t2 = (x) - 1020
            t3 = (x) - 1100
            t4 = (y) - 50
            t5 = (y) - 125
            t6 = (y) - 375
            t7 = (y) - 450

            # Setting the line constrain to obatain the obstacle space with buffer
            if((h6>0 and h5>0 and h4<0 and h3<0 and h2<0 and h1>0) or (r11>0 and r12>0 and r13<0 and r14<0 ) or (r21>0  and r23<0 and r24<0 and r22>0) or (t1>0 and t2<0 and t4>0 and t5<0) or (t2>0 and t3<0 and t4>0 and t7<0) or (t6>0 and t7<0 and t1>0 and t2<0) or (w1<0) or (w2>0) or (w3<0) or (w4>0)):
                obstacle_space[y, x] = 2    

    return obstacle_space


#Function to check if the Goal Node is Reached.

def check_goal(current, goal): 
    
	if (current.x == goal.x) and (current.y == goal.y):
		return True
	else:
		return False


# Funtion to check if the move is valid.

def check_valid(x, y, obstacle_space):
    
    # Obstacle space limit
	size = obstacle_space.shape
    
    # Check the Boundary 
	if( x > size[1] or x < 0 or y > size[0] or y < 0 ):
		return False
    
	# Check if the given node is in Obstacle Space
	else:
		try:
			if(obstacle_space[y][x] == 1) or (obstacle_space[y][x] == 2):
				return False
		except:
			pass
	return True