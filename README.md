# 2D Rapidly-exploring Random Tree (RRT) for Robot Path Planning
Author: Muye Jia

This RRT algorithm allows for 2D path planning for robots. The tree can find a free path around obstacles
with any shapes and sizes, and the path along with the Tree generated is visualized using Matplotlib.

Input a map image file with 1 representing obstacles and 0 representing free space.
The output would be a path stored as csv file named `path.txt` and a mp4 file `ani.mp4` with planning animation.

## Example
The algorithm is going to plan a path around the Northwestern-N-shaped obstacle. Starting at (40, 40), ending at (60, 60).
The resulted path (plot in red):


![image](https://user-images.githubusercontent.com/112987403/209891484-771e161f-f692-4da8-b355-623d5ccb1f65.png)

The RRT expansion animation:

https://user-images.githubusercontent.com/112987403/209891383-664a276e-0bc8-45ee-9b3f-a06829334f96.mp4



# Quick Start Guide
1. To run the algorithm with customized obstacle map (or the N_map.png in the repo), run the following command:
`python3 obstacle_avoidance.py <map_name> <tree_step_size> <planning_domain> <max_node_number> <start_x_position> <start_y_position> <goal_x_position> <goal_y_position>`
 
  Example run: `python3 obstacle_avoidance.py N_map.png 1 100 500 5 5 40 40`.

2. Note: The input map file needs to be in 1 and 0, where 1 represents the obstacle in map and 0 represents free space.

# Required Packages
Need `FFMpeg` for Matplotlib animation.
