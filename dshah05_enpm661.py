# Importing Necessary Libraries
import numpy as np
import matplotlib.pyplot as plt 
import time
import heapq
from matplotlib.animation import FuncAnimation
import cv2
 

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
            h6_buffer = (y + 5) +  0.58*(x + 5) - 475.098
            h5_buffer = (y + 5) - 0.58*(x - 5) + 275.002
            h4_buffer = (x - 6.5) - 779.9
            h3_buffer = (y - 5) + 0.58*(x - 5) - 775.002
            h2_buffer = (y - 5) - 0.58*(x + 5) - 24.92
            h1_buffer = (x + 6.5) - 520.1
            
            # Block Obstacle
            t1_buffer = (x + 5) - 900
            t2_buffer = (x + 5) - 1020
            t3_buffer = (x - 5) - 1100
            t4_buffer = (y + 5) - 50
            t5_buffer = (y - 5) - 125
            t6_buffer = (y + 5) - 375
            t7_buffer = (y - 5) - 450
            
            # Setting the line constrain to obatain the obstacle space with buffer
            if((t1_buffer>0 and t2_buffer<0 and t4_buffer>0 and t5_buffer<0) or(t2_buffer>0 and t3_buffer<0 and t4_buffer>0 and t7_buffer<0) or (t6_buffer>0 and t7_buffer<0 and t1_buffer>0 and t2_buffer<0) or (r11_buffer>0 and r12_buffer>0 and r13_buffer<0 and r14_buffer<0) or (r21_buffer>0 and r23_buffer<0 and r24_buffer<0 and r22_buffer>0) or (h6_buffer>0 and h5_buffer>0 and h4_buffer<0 and h3_buffer<0 and h2_buffer<0 and h1_buffer>0)):
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
            h6 = (y) +  0.58*(x) - 475.098
            h5 = (y) - 0.58*(x) + 275.002
            h4 = (x) - 779.9
            h3 = (y) + 0.58*(x) - 775.002
            h2 = (y) - 0.58*(x) - 24.92
            h1 = (x) - 520.1 
            
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

# Generating a Unique ID

def unique_id(node):
    id = 3333*node.x + 113*node.y 
    return id

# Dijkstra Algorithm

def dijkstra_algorithm(start, goal, obstacle_space):
    
    ## Check if given Node is the Goal Node
    if check_goal(start, goal):
        return None, 1
    
    # Storing as Nodes only when Start is not Goal
    goal_node = goal
    start_node = start
    
    # Data Handlers
    unexplored_nodes = {}
    all_nodes = []
    explored_nodes = {} 
    open_list = []
    
    # All possible Movements for a Node
    possible_moves = ['Up','UpRight','Right','DownRight','Down','DownLeft','Left','UpLeft']
        
    # Assigning Unique Key to Start Node
    start_key = unique_id(start_node)
    unexplored_nodes[(start_key)] = start_node
    
    # Push the Start Node into the Heap Queue
    heapq.heappush(open_list, [start_node.cost, start_node])
    
    # Loops until all the Nodes have been Explored, i.e. until open_list becomes empty
    while (len(open_list) != 0):
        
        current_node = (heapq.heappop(open_list))[1]
        all_nodes.append([current_node.x, current_node.y])
        current_id = unique_id(current_node)

        if check_goal(current_node, goal_node):
            goal_node.parent_node = current_node.parent_node
            goal_node.cost = current_node.cost
            print("Goal Node found")
            return all_nodes,1

        if current_id in explored_nodes:
            continue
        else:
            explored_nodes[current_id] = current_node
		
        del unexplored_nodes[current_id]
        
        # Looping through all possible nodes obtained through movements
        for moves in possible_moves:
            
            # New x,y,cost after moving.
            x,y,cost = move_node(moves, current_node.x, current_node.y, current_node.cost)
            
            # Updating Node with the New Values
            new_node = Node(x, y, cost, current_node)
            
            # New ID
            new_node_id = unique_id(new_node)
            
            # Checking Validity
            if not check_valid(new_node.x, new_node.y, obstacle_space):
                continue
            elif new_node_id in explored_nodes:
                continue
            
            # Updating Cost and Parent, if lesser cost is found.
            if new_node_id in unexplored_nodes:
                if new_node.cost < unexplored_nodes[new_node_id].cost: 
                    unexplored_nodes[new_node_id].cost = new_node.cost
                    unexplored_nodes[new_node_id].parent_node = new_node.parent_node
            else:
                unexplored_nodes[new_node_id] = new_node
   			
            # Pushing all the open nodes in Heap Queue to get the least costing node.
            heapq.heappush(open_list, [ new_node.cost, new_node])
   
    return  all_nodes, 0

def back_track_path(goal_node):
    
    # Creating a Stack to hold the Backtracked Nodes
    backtrack_stack = []
    
    # Storing the Goal Node
    backtrack_stack.append(goal_node)
    
    # Backtracking to the Preceeding Nodes until Start Node is Reached
    while backtrack_stack[-1].parent_node != -1:
        parent = backtrack_stack[-1].parent_node
        backtrack_stack.append(parent)
        
    # Since the Backtracking was done from Goal to Start Node, Reversing the Stack to get path from Start to Goal
    backtrack_stack.reverse()
    
    # Extracting the x, y coordinates of the path from the Backtracked Nodes
    x_path = [node.x for node in backtrack_stack]
    y_path = [node.y for node in backtrack_stack]
    
    return x_path, y_path

class PathPlotter:
    def __init__(self, start_node, goal_node, obstacle_space):
        self.start_node = start_node
        self.goal_node = goal_node
        self.obstacle_space = obstacle_space
        
    def animate_exploration_and_path(self, obstacle_space, explored_nodes, x_path, y_path):
        fig, ax = plt.subplots()
        ax.imshow(obstacle_space, cmap="magma")
        
        # Plot start and goal positions
        ax.plot(self.start_node.x, self.start_node.y, "Db")
        ax.plot(self.goal_node.x, self.goal_node.y, "Dg")
        
        # Plot the exploration
        exploration, = ax.plot([], [], 'r*')
        
        # Plot the path
        line, = ax.plot([], [], 'w--')
        
        # Set up the axes properties
        ax.set_xlim(0, 1200)
        ax.set_ylim(0, 500)
        batch_size = 850
        
        def update(frame):
            # Update the data of the line for each frame
            line.set_data(x_path[:frame+1], y_path[:frame+1])
            
            # Update the exploration every 750 nodes
            
            start_index = batch_size * frame
            end_index = min(batch_size * (frame + 1), len(explored_nodes))
            exploration.set_data([node[0] for node in explored_nodes[start_index:end_index]], 
                                 [node[1] for node in explored_nodes[start_index:end_index]])
            
            return exploration, line
        
        # Calculate the total number of frames
        total_frames = (len(explored_nodes) + batch_size - 1) // batch_size
        
        # Create the animation
        ani = FuncAnimation(fig, update, frames=total_frames, interval=2, blit=True)
        plt.show()

# Main Function to Execute the Program
if __name__ == '__main__':
    
    # Robot Space width and Height
    width = 1200
    height = 500
    
    # Getting the Coordinates of Obstacles
    obstacle_space = plot_map(width, height)
    
    # Getting the Start and Goal Coordinates from the User    
    start_x = int(input("Enter Start Position's X Coordinate: "))
    start_y = int(input("Enter Start Position's Y Coordinate: "))

    goal_x = int(input("Enter Goal Position's X Coordinate: "))
    goal_y = int(input("Enter Goal Position's Y Coordinate: "))
    
    # Start Timer to Check the Time of Execution
    start_time = time.time()
    
    # Validating the Start Node if it is in Permitted Robot Space
    if not check_valid(start_x, start_y, obstacle_space):
        print("Enter Start Node within the permitted Robot Space")
        exit(-1)
    
    # Validating the Goal Node if it is in Permitted Robot Space
    if not check_valid(goal_x, goal_y, obstacle_space):
        print("Enter Goal Node within the permitted Robot Space")
        exit(-1)
    
    ## Creating Start Node which has attributes start (x,y) coordinates, cost = 0 and parent_node = -1
    start_node = Node(start_x, start_y, 0.0, -1)
    
    ## Creating Goal Node which has attributes goal (x,y) coordinates, cost = 0 and parent_node = -1
    goal_node = Node(goal_x, goal_y, 0.0, -1)
    
    # Implementing Dijkstra Algorithm
    explored_nodes, goal_status = dijkstra_algorithm(start_node, goal_node, obstacle_space)
    
    # Back Track only if the Goal Node is reached.
    if (goal_status == 1):
        x_path, y_path = back_track_path(goal_node)
        
        # Plotting the Map, Obstacles and the Generated Path
        plotter = PathPlotter(start_node, goal_node, obstacle_space)
        plotter.animate_exploration_and_path(obstacle_space, explored_nodes, x_path, y_path)
        cost = goal_node.cost
        print(f"Cost of reaching the goal: {cost:.2f}")
    else:
        print("Goal could not be planned for the given points")

    # Stopping the Timer after Program Execution
    end_time = time.time()
    print(f"Time Taken to Execute the Program is: {end_time - start_time}")


