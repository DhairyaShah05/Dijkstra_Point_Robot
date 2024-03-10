# Importing Necessary Libraries
import numpy as np
import matplotlib.pyplot as plt 
import time
import heapq
from matplotlib.animation import FuncAnimation

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
    if start.x == goal.x and start.y == goal.y:
        return None, 1

    goal_node = goal
    start_node = start
    unexplored_nodes = {}
    all_nodes = []
    explored_nodes = {}
    open_list = []
    possible_moves = ['Up', 'UpRight', 'Right', 'DownRight', 'Down', 'DownLeft', 'Left', 'UpLeft']

    start_key = unique_id(start_node)
    unexplored_nodes[start_key] = start_node
    heapq.heappush(open_list, [start_node.cost, start_node])

    while open_list:
        current_node = heapq.heappop(open_list)[1]
        all_nodes.append([current_node.x, current_node.y])
        current_id = unique_id(current_node)

        if current_node.x == goal_node.x and current_node.y == goal_node.y:
            goal_node.parent_node = current_node.parent_node
            goal_node.cost = current_node.cost
            print("Goal Node found")
            return all_nodes, 1

        if current_id in explored_nodes:
            continue
        else:
            explored_nodes[current_id] = current_node
        
        del unexplored_nodes[current_id]
        
        for move in possible_moves:
            x, y, cost = move_node(move, current_node.x, current_node.y, current_node.cost)
            new_node = Node(x, y, cost, current_node)
            new_node_id = unique_id(new_node)
            
            if not check_valid(new_node.x, new_node.y, obstacle_space) or new_node_id in explored_nodes:
                continue
            
            if new_node_id in unexplored_nodes:
                if new_node.cost < unexplored_nodes[new_node_id].cost: 
                    unexplored_nodes[new_node_id].cost = new_node.cost
                    unexplored_nodes[new_node_id].parent_node = new_node.parent_node
            else:
                unexplored_nodes[new_node_id] = new_node
            
            heapq.heappush(open_list, [new_node.cost, new_node])
    
    return all_nodes, 0

# Function to Back Track position from Goal Node to Start Node.
    
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

#This class is used to plot and see the explored nodes.

class PathPlotter:
    def __init__(self, start_node, goal_node, obstacle_space):
        self.start_node = start_node
        self.goal_node = goal_node
        self.obstacle_space = obstacle_space
        
        # Plot the Start and Goal Positions
        plt.plot(start_node.x, start_node.y, "Db")
        plt.plot(goal_node.x, goal_node.y, "Dg")
        
        # Plot Map
        plt.imshow(obstacle_space, "seismic")
        self.ax = plt.gca()
        self.ax.invert_yaxis()
        
        # Store explored nodes for batch plotting
        self.explored_nodes = []
        self.explored_batch_size = 100  # Adjust batch size as needed

    def plot_explored_nodes(self, explored_nodes):
        self.explored_nodes.extend(explored_nodes)
        
        # Plot in batches for faster performance
        if len(self.explored_nodes) >= self.explored_batch_size:
            self._plot_batch()

    def _plot_batch(self):
        # Convert explored nodes to numpy array for vectorized plotting
        explored_array = np.array(self.explored_nodes)
        x_coords, y_coords = explored_array[:, 0], explored_array[:, 1]
        plt.plot(x_coords, y_coords, "3y")
        
        # Clear explored nodes after plotting
        self.explored_nodes = []

    def plot_path(self, x_path, y_path):
        plt.plot(x_path, y_path, "--r")
        # plt.show()
        plt.close('all')

    def animate_exploration_and_path(self, obstacle_space, explored_nodes, x_path, y_path):
        fig, ax = plt.subplots()

        # Plot the exploration
        ax.imshow(obstacle_space, "seismic", origin='lower')  # Set origin to lower for consistency

        # Plot the explored nodes
        line_explore, = ax.plot([], [], '3y', markersize=2)  # Marker for explored nodes

        # Plot the path line (empty initially)
        line_path, = ax.plot([], [], '--r', linewidth=2)  # Path line

        # Set up the axes properties
        ax.set_xlim(0, 1200)
        ax.set_ylim(0, 500)

        # Find the shorter path
        shorter_path_length = min(len(x_path), len(y_path))

        # Linearly interpolate the longer path to match the length of the shorter path
        if len(x_path) > shorter_path_length:
            x_path_interp = np.interp(np.linspace(0, 1, shorter_path_length), np.linspace(0, 1, len(x_path)), x_path)
            y_path_interp = np.interp(np.linspace(0, 1, shorter_path_length), np.linspace(0, 1, len(y_path)), y_path)
        else:
            x_path_interp, y_path_interp = x_path, y_path

        def update(frame):
            # Plot the path line frame by frame
            if frame < shorter_path_length:
                line_path.set_data(x_path_interp[:frame + 1], y_path_interp[:frame + 1])
            else:
                line_path.set_data([], [])

            # Plot explored nodes if it's a multiple of 20 or if it's the last frame
            if frame % 500 == 0 or frame == shorter_path_length - 1:
                if frame < len(explored_nodes):
                    line_explore.set_data([node[0] for node in explored_nodes[:frame + 1]],
                                        [node[1] for node in explored_nodes[:frame + 1]])
                else:
                    line_explore.set_data([], [])

            return line_explore, line_path

        # Create the animation with frames equal to the length of the shorter path
        ani = FuncAnimation(fig, update, frames=shorter_path_length, interval=5, blit=True)
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
        plotter.plot_explored_nodes(explored_nodes)
        plotter.plot_path(x_path, y_path)
        # plotter.animate_exploration_only(obstacle_space,explored_nodes)
        plotter.animate_exploration_and_path(obstacle_space, explored_nodes, x_path, y_path)
        cost = goal_node.cost
        print(f"Cost of reaching the goal: {cost:.2f}")
    else:
        print("Goal could not be planned for the given points")

    # Stopping the Timer after Program Execution
    end_time = time.time()
    print(f"Time Taken to Execute the Program is: {end_time - start_time}")