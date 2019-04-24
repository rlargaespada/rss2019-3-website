# Lab 6: Path Planning and Trajectory Tracking

# Table of Contents

## 1) Overview and Motivation (Raul)
The planning and execution of collision-free paths within an environment is an essential part of autonomously operating a robot to complete complex tasks. Effective path planning requires combining path planning algorithms with localization and pure pursuit control algorithms we have previously developed, and as such represents the culmination of much of our prior coursework. To build upon what we have learned and enable our racecar to complete our most advanced challenge thus far, we set about implementing and following path planning algorithms on our system.

We wanted to evaluate two separate classes of planning algorithms: sampling-based algorithms which randomly sample the state space and construct a path by exploring through the samples, and search-based algorithms which explore the possible robot state space to generate an optimal path from start to goal. We implemented and tested the RRT* (rapidly-exploring random trees) sampling algorithm and the A* search algorithm in simulation, and after evaluating the performance of both, deployed RRT* algorithm on our racecar. By using RRT* we were able to quickly generate and follow paths through the Stata basement.

## 2) Sampling-Based Path Planning (Abbie)
We considered three different sampling-based planning algorithms for this task, PRM, RRT, and RRT*. Properties of these algorithms, such as asymptotic optimality (whether it finds an optimal path as the number of samples goes to infinity), query structure (whether one construction can be used to find solutions to multiple problems, search requirement (whether the graph will require search, such as A* or BFS after construction), and dynamics (whether we can incorporate a motion model of the car into the planning phase).

| | Asymptotically Optimal | Single- or Multi- Query | Search after Construction | Dynamics |
| :--- |   :---   | :--- | :--- | :---: |
| **PRM**| Yes| Multi | Required | Difficult to incorporate |
| **RRT**| No| Single | Retrace from goal | Explicit in construction |
| **RRT***| Yes| Single | Retrace from goal | Explicit in construction |
$\footnotesize \textbf{Table 1 - Sampling method comparisons } \text{A comparison of the different aspects of planning algorithms considered in chosing a sampling algorithm. }$

We chose to move forward with RRT* because it allows us to incorporate dynamics into the system and is asymptotically optimal. Although PRM requires one construction and can be queried multiple times for different paths, a search is required each time, and therefore concluded PRM wold likely include an advantage in runtime.

### 2.1) RRT: A Suboptimal Solution (Alex)
The basis of RRT* lies in the more basic and non-optimal.  The RRT algorithm takes in a start and initial goal, and begins with one node at the start goal.  We then sample randomly at a point on the map, and locate the closest existing node as a potential parent.  From this potential parent, we check if the racecar can steer the towards the sampled point by a set distance (0.5m in our case) without collision.  If a collision would occur, we can ignore this sampled point.  However, if no collision occurs, create a new node at the endpoint of this steering.  Implementing the steer functionality can take into account the dynamics of the robot either through rough approximations of Ackermann Steering, Dubins curves, or other methods.  However, we simply created straight lines from one node to the steer position.  We made this decision due to our controller's proficiency at sharp turns, improving runtime, and that the area we tested (the Stata basement) generally involves paths with a few sharp turns instead of dynamically complex obtainable avoidance.  To further compensate for the lack of dynamics in our RRT implementation, we increase the width of map edges by .4 m to keep any paths from getting too close to walls and corners during turns.  Additionally, we define a a region surrounding the goal pose from which we sample points at a higher rate in order to pull the tree in the goal's general direction.  In our implementation, RRT samples a random point from the full map with a probability 0.8 and a point from the goal region with a probability of 0.2. Once a node reaches the goal region, we connect it directly to the goal pose, and traverse the tree through the node's parents back to the start node to create the path.

---INSERT RRT.JPEG HERE --
$\footnotesize \textbf{Figure 1 - RRT Path } \text{An RRT Tree (in blue) and path found (in white) on a map of stata basement.  Note its suboptimal path and many kinks.}$

### 2.2) RRT*: Finding optimal paths (Abbie)
The difference between RRT and RRT* lie in a "rewiring" function that chooses the optimal path at each step along which to connect the new node. This rewiring happens in two phases.

First, for each new node, we want to find the parent within a certain neighbor radius such that the cost from the start to that new node is minimize. To do this, we iterate through all the neighbors of our new node, and set its parent to be the neighbor with the minimum $cost(\text{start} \rightarrow \text{neighbor})+ cost(\text{neighbor} \rightarrow \text{current})$.

The second phase involves optimizing already existing edges in the tree. Again, we iterate through the neighbors of the new node, and if an existing path can be improved by going through the new node, we make that update. For each neighbor, if $cost(\text{start} \rightarrow \text{neighbor}) > cost(\text{start} \rightarrow \text{current})+ cost(\text{current} \rightarrow \text{neighbor})$, we set the parent of that neighbor to be the current node.

These two updates ensure that at every step, we are maintaining the most optimal paths found so far in our tree. RRT* is asymptotically optimal, meaning the path converges on the optimal solution as the number of samples goes to infinity. However, this is inefficient in practice, so in our implementation, we count the number of iterations necessary to find a path to the goal, $n$, and continue to run the optimization for an addition $0.5*n$ steps. 

To enable quick nearest neighbors search, we store each node in an R-Tree, a data structure that clusters points in rectangles based on cartesian coordinates.

#### Path Smoothing
As a final step to make out returned trajectories dynamically feasible, we smooth the paths returned by RRT* by checking if the cost of the path can be improved by connecting each node to one of its ancestors within a certain radius rather than its parents. Although this gives us fewer points around corners, we found both in simulation and in real life that this was not a problemstill allowing  to generate eicient paths.

**INSERT RRT STAR.JPEG**
$\footnotesize \textbf{Figure 1 - Smoothed RRT* Path } \text{An RRT* Tree (in blue) and path found (in white) on a map of stata basement.} \\ \text{Smoother than the RRT path, and thus better for our pure pursuit controller.}$

## 3) Search-Based Path Planning (Kayla)
Out of the three search algorithms proposed to us (BFS, Dijkstra's, and A*), A* was the best option for our search space. While all three are effective algorithms, BFS and Dijkstra's algorithm each have some shortcomings. BFS is able to guarantee finding the shortest path nodes a by expanding frontiers of nodes in a graph, but because frontiers are expanded uniformly, the search space is very large so it's very slow. Similarly, Dijkstra's algorithm finds the shortest path in a graph but spends unnecessary time exploring paths that aren't very promising. A* combines a heuristic that expands the frontier more towards the goal with Dijkstra's algorithm to offer an algorithm that will return the shortest path without unnecessary deviations. 

### 3.1 Sample Space Discretization (Raul and Kayla)
A* requires a graph to search through, so we needed to convert the map representation from an occupancy grid to a graph. Unoccupied cells in the occupancy grid would correspond to nodes in the graph, with neighboring cells acting as neighboring nodes. In the graph, nodes are represented with $(x,y)$ coordinates in the map frame in unites of meters.  In our initial implementation, we had a direct 1:1 correspondence between graph nodes and occupancy grid cells, resulting in a very large graph that took significant time to build and search through. To improve upon this, we changed our graph building algorithm to space nodes out by varying distances, and found that a spacing of $0.5m$ provided an acceptable runtime while still allowing A* to generate efficient paths.

An additional challenge was converting between occupancy grid indexes and physical coordinates in the map. To do this, we multiplied the occupancy grid indexes are represented with (x, y) coordinates in the map frame (in meters). To convert between the occupancy grid coordinates and the map coordinates, the occupancy grid coordinates are multiplied by the map resolution of $0.05 m$ per cell, times $-1$ to account for the new axes directions. We then add the coordinates of the map's origin to get the coordinates in the correct frame, multiplied by $-1$, then the origin of the map is added to that. 
### 3.2 A* (Kayla)
A* finds the path to a goal location by using a search frontier that expands towards the goal. I does this with the definition of a cost function that calculates the cost between the start and current node in the search and a heuristic, which approximates the cost between the current node and the goal point. The heuristic and cost function combined determine the order in which nodes in the graph are explored. A* works optimally when the graph has well defined costs between nodes and a good heuristic. A* favors movements that minimize both the cost from the start state and the estimated cost to the goal state, given by the chosen heuristic. Each state is assigned a total approximate cost given by
$$f(x) = c(x) + h(x) \quad \quad \text(1)$$ with $c(x)$ as the cost from the start and $h(x)$ as the cost from the heuristic. Varying the heuristic and cost functions becomes a trade off between speed and accuracy. If the heuristic is completely accurate to the path, A* will only expand the nodes along the path and the algorithm is completely optimal. If the heuristic is lower than the true value, the algorithm will run slower but is guaranteed to find the optimal path. If the heuristic overestimates, A* will expand towards the goal faster but won't necessarily find the shortest path.

For our heuristic, we chose to use true distance between the current point and the goal point: $$d = \sqrt {\left( {x_1 - goal_x } \right)^2 + \left( {y_1 - goal_y } \right)^2 } \quad \text(2)$$ Because the Stata basement is mostly square shaped with long straight sections, true distance works well as a heuristic. However there are situations where the true distance heuristic causes A* to explore nodes sub-optimally.

---**insert rASTAR.pngr here**---

## 3) Pure Pursuit Controller (Kyle)
### 3.1 Overview
In this lab, we implemented a pure pursuit controller to follow the path provided by the path planning algorithms. Once the path is received from the path planner and the robot's position within the map frame is received from the localization node, the closest point to the robot is found. The path is then sliced to 25 points out from the nearest way-point. This is done for two reasons. First, it ensures the robot will always track points ahead of it, as points it has already passed are no longer tracked. Second, it reduces the size of the path that the node must keep track of, keeping computation time short. If there are not 25 points left in the path, the robot wraps the next 25 points from the start of the path, which allows it to successfully run laps around a cyclic path. Next, the lookahead distance is dynamically determined as a function of speed. We identified three key speed regimes: Slow/Testing, Normal Operation, and Fast.

### 3.2 Goal Point Selection

Once the lookahead distance has been set, the robot identifies the two way-points straddling the lookahead distance. From this computation, three cases arise. The first is standard operation. The lookahead distance is between two future way-points, and the new x and y coordinates are calculated for input into the pure pursuit controller. If the path is sparse and the distance between the robot and the next way-point is longer than the lookahead distance, then the car calculates the x and y coordinate along the line segment connecting the robot's current pose with the next known way-point. Finally, if the robot is more than one lookahead distance away from the path, then the lookahead distance is reset as the distance between the robot and the first point on the path, and that way-point is input into the controller. The new x and y coordinates are computed geometrically to avoid slope errors that may arise if the robot is traveling parallel to the x or y axes. Finally, the desired x and y coordinates are input into the pure pursuit controller, which uses Ackerman Steering to set the wheel angle to hit the desired point via a circular arc. This entire process is run at the same rate as localization.
## 4) ROS Implementation (Alex)
Two main nodes run in our implementation of path planning and following.  The first is /PureP which is the pure pursuit controller.  /PureP listens for a path to follow over the topic /test_path which is sent either byto /rrt (when when running the sampling planner) or /astar (when running the search planner).  In addition to the nodes and topics used for path following and our controller, we run the /particle_filter node in order to obtain the racecar's approximate position and send it to the controller via /estim_pose. / for the path as a PointCloud via the topic /test_path.  /PureP then publishes a drive command to /vesc/ackermann_cmd_mux/input/nav_1.  /PureP also listens  a Point32 message over the topic /estim_pose as for the pose according to the particle_ filter is informed by the /map, /scan, and /vesc/odom topics.  

--INSERT RQT_graph.PNG HERE--
$\footnotesize \textbf{Figure 3 - RQT graph} \text{A graph representing the ROS nodes and topics discussed above }$
.  Additionally, we have several visualization topics.  /rrt_tree publishes a visualization of the tree built by RRT*. 

## 5) Simulation Results
Algorithms were evaluated based on multiple characteristics to choose which was the best suited to be put on the car. Most significant were overall runtime, path length, and path optimality given the car's controller (measured qualitatively). The pure-pursuit controller was evaluated on its ability to stay on the given path using the distance from the path as measured error. 
### 5.1 RRT*
We ran RRT, RRT*, and RRT* with path smoothing between the same two waypoints and calculated these average costs over 10 trials.

| | RRT | RRT* | RRT* with path smoothing |
| :--- |  :--- | :--- | :----:   | 
| **Average Cost (meters)**| 70.75 | 69.41 | 69.25 |
$\footnotesize \textbf{Table 2 - Cost comparison for RRT algorithms }$

While the difference in cost is not significant, particularly between RRT* and RRT* with smoothing, RRT* with smoothing ultimately returned the trajectory that resulted in the best performance with our controller.

Finally, we used RRT* to plan a full loop around the Stata basement

| | Averaged Metrics for a Full Stata Basement Loop using RRT* |
| :--- |    :----:   | 
| **Runtime**| 80 s |
| **Path Length**| 121 m |
$\footnotesize \textbf{Table 3 - RRT* full loop performance}$

RRT* returns close to an optimal path, while taking significantly less time than our current implementation of A*. 

### 5.2 A* (Kayla)

| | Averaged Metrics for a Full Stata Basement Loop using A* |
| :--- |    :----:   | 
| **Building the Graph**| 154 s |
| **Running A***| 2 s |
| **Path Length**| 120 m |
$\footnotesize \textbf{Table 2 - Runtime comparisons } \text{A comparison of the different runtimes and returned path length of our planning algorithms }$

Our implementation of the A* algorithm was able to successfully find the shortest path in the Stata basement and on graphs of our own creation. When we broke down the overall runtime into building the graph and running A*, we found that running A* takes significantly less time than building the actual graph.

### 5.3 Controller
| | Avg Dist to Path (m)| 
| :--- |    :----:   | 
| **Overall Path**| 0.0057|
| **Around Corners**| 0.2| 
$\footnotesize \textbf{Table 3 - Controller Accuracy in Simulation } \text{The accuracy of our pure pursuit controller in simulation as measured by the average distance from the desired path for an overall path and turns}$

The simulation utilizes our implementation of RRT as well as localization to produce the desired path and robot's current pose for use in the controller. We found that at low to mid speeds ( less than 5-6 meters per second) we were able to match the path exactly, with the robot only diverging from the path around sharp corners, but correcting itself within 1 meter. This high fidelity controller gives us the confidence to test the robot at higher speeds, once we have implemented a speed controller to slow the robot around corners to ensure no risk of hitting the wall.

## 5) Experimental Results

We measured the controller in the real world in much the same way as in the simulation.  We keep track of the minimum distance from the estimate from the particle filter and the path.  However, it should be noted that the particle filter estimation of the pose is not entirely accurate.  However, the error should be rather negligible and not affect the average over a long period of time since it should not cause error specifically towards or away the path.  

| | Avg Dist to Path (m)| 
| :--- |    :----:   | 
| **Overall Path**| 0.022|
| **Around Corners**| 0.21|
$\footnotesize \textbf{Table 4 - Real world controller results } \text{The accuracy of our pure pursuit controller in simulation as measured by the average distance from the desired path for an overall path and turns}$
The real world controller works with less accuracy in simulation.  We expect a decrease in accuracy simply due to the dynamics of the real world.  On a more qualitative level, the controller will often cause the steering to shake and oscillate very quickly.  While this results in momentary instability, it does not generally decrease the effectiveness of the controller's ability to follow a line.  Additionally, the controller will occasionally drift from the path and entirely diverge.  We still cannot identify the reason for this, though it may be an error in the way in which we look ahead to future points on the path.  However, 

## 6) Future Work

### 6.1 Planning Algorithm (Raul)
Comparing the speed at which the A* algorithm runs to the time spent building the Stata basement map graph, we find a clear area for improvement. While it would be difficult to decrease the graph building runtime without sacrificing a large amount of resolution, other options exist, such as pre-computing the graph and pulling the appropriate data from a file before running A*. Because the map is static, this would eliminate nearly all of the runtime without any loss in performance. Additional work could go into determining how much resolution we can add to the map before it becomes difficult to build and save or A*'s runtime becomes excessive. 

Additionally, our A* algorithm uses a heuristic of Euclidean distance between the current search point and the goal point. This is an acceptable heuristic for most start/goal pairs, but it does not take into account racecar dynamics or the various curves of the basement loop. For future improvements we will consider different heuristics, such as using a Dubins curve to track distance to the goal instead of straight line distance.

### 6.2 Controller
In order to allow the robot to traverse the path at higher speeds for the race, we intend to make two key changes to the controller. First, we intend to implement a speed controller to dynamically change the speed of the robot given the path, allowing the robot to slow down around corners and speed up through straightaways. In order to do so, the robot will fit a linear regression to the next 5 meters of the path in front of the car. The robot's speed will then be set as a function of the R squared goodness of fit parameter of the linear regression. The second key change will be changing the lookahead distance to be a function of the path's curvature instead of the robot's velocity. This will be calculated in a similar way to the speed controller. These two changes will allow the car to more effectively traverse the race course at a higher speed.