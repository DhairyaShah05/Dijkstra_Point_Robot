# Path Planning with Dijkstra Algorithm

This project implements path planning using the Dijkstra algorithm in Python. The algorithm is applied to a grid-based environment with obstacles, finding the shortest path from a start point to a goal point while avoiding obstacles.

## Table of Contents

- [Introduction](#introduction)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [File Structure](#file-structure)
- [Author](#author)

## Introduction

Path planning is a crucial aspect of autonomous robotics, enabling robots to navigate efficiently in complex environments. The Dijkstra algorithm is a popular graph search algorithm used for finding the shortest path between nodes in a graph. In this project, the Dijkstra algorithm is applied to a grid environment, where each cell represents a node, and obstacles are present in some cells. The algorithm calculates the shortest path from a start cell to a goal cell while avoiding obstacles.

## Dependencies

- `numpy`: Used for numerical calculations and array manipulation.
- `matplotlib`: Utilized for visualization and plotting.
- `opencv-python`: Used for image processing and animation.

## Installation

1. **Install Dependencies:**
   Ensure you have Python installed on your system. You'll also need to install the required libraries. You can do this via pip:
   ```bash
   pip install numpy matplotlib opencv-python
   ```

## Usage

1. **Run the Code:**
   Execute the Python script `dshah05_enpm661.py`:
   ```bash
   python dshah05_enpm661.py
   ```

2. **Input Start and Goal Coordinates:**
   When prompted, enter the coordinates for the start and goal positions in the grid environment. Ensure that the coordinates are within the permissible robot space and not obstructed by obstacles. The origin of the map is on the bottom left corner. 

3. **Visualization:**
   The code will generate a visualization of the explored nodes during the algorithm's execution and the final path found from start to goal. Where the red region shows the area of explored nodes for the given input and the dotted white line will be the shortest path between the start and the goal point. (Note : The visualization speed is tuned for a specific case i.e. Start Co-ordinates : (5,495) Goal Co-ordinates : (1195,5) (From Top left to Bottom Right)

## File Structure

- `dshah05_enpm661.py`: Main Python script containing the Dijkstra algorithm implementation and visualization code.
- `README.md`: This README file providing instructions and information about the project.
- `sketch.sldprt`: This is the sketch file of the map in SolidWorks. Used to determine co-ordinates of points for deriving Half-plane equations.
- `dshah05_dhairya_shah.mp4`: This is the sample execution video file that you can refer to for implementing in your machine.
 
## Author

- Dhairya Shah
- dshah05@umd.edu

## License

This project is licensed under the [License Name] License - see the [LICENSE](LICENSE) file for details.

---

Feel free to customize the README file further with additional information or sections as needed for your project.
