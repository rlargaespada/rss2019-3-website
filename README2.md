# Final Challenge: High Speed Object Avoidance

# Table of Contents
1. Overview and Motivation (Raul)
2. Strategies
	1. Mapping and A* (Raul)
	2. Greedy Optimal Heading
3. Comparison of Strategies
4. Parameter Tuning
	1. Particle Filter Tuning
	2. Algorithm Tuning (Raul)
5. ROS Implementation 
6. Experimental Results
7. Future Work

## 1) Overview and Motivation 
 - Fast obstacle avoidance is an important problem in robotics
 - Challenges are increased when obstacle locations are not known beforehand
 - Wanted to develop a robust algorithm that would be able to navigate through a complex field of obstacles on the path towards a goal
 - Anything else?

## 2) Strategies and Ideas

### 2.1) Mapping and A*

 - Initial idea was to start with a graph representing the Stata basement and run A* from start to goal
 - As obstacles were detected by the laser scan, nodes in the graph corresponding to those positions with obstacles would be removed and A* would be run again
 - This process would potentially be augmented with specific mapping tools such as gmap
 - We decided/discovered that this process would be too slow for real-time avoidance and switched to an algorithm that did not involve mapping.

#### Possible section: Dubins path planning? (Raul)

### 2.2) Greedy Optimal Heading
A second option for our algorithm involves greedy decision making rather than planning.  This algorithm checks sections of the laser scan data closest to the direction of the goal and makes two layers of checks to find the most fitting angle.  First, we check a large angle of scans around one such section and check the number of scans which fall under a short threshold.  If too many scans fall under this threshold, we mark the section as having immediate danger and remove it from the Racecar's possible headings.  If a section passes this first layer, we analyze a narrower slice and assign a score based on 3 parameters.  Firstly, we check the proportion of numbers greater than some distant threshold (approximately three times the distance of the first).  Second, we check the angle between the proposed section and the scan the goal.  This parameter ensures we consistently move toward the goal as much as possible.  Finally, we check the angle between the current heading and proposed section of the scan, which helps maintain the stability of our path and reduce oscillations which slow progress and can lead to collision.  Weighting and comparing these scores across all the sections gives us an optimal direction in which to steer.  We then use an stabilized Ackermann steering model to approach the point chosen.  **opison of ratees  aramete in  rtce tee** (fix the bold part, I think it got messed up somehow) our algorithm relies largely on the laser scan data alone, having a rudimentary understanding of our location on the map remains necessary.  Specifically, maintaining a general understanding of our heading was important for the purposes of knowing our angle to the goal.  However, while our localization proved reliable when in a known environment, the obstacle field naturally included many unknown and unmapped obstacles, throwing the localization wildly off during testing.  To mitigate these problems, we modified the particle filter's probability distribution to reflect the introduction of new obstacles.  Specifically, we raised the weight for the probability of a laser scan to measure a distances shorter than expected from .07 to .40, and lowered the probability of a distance approximately equal to that expected from the map from .74 to .41.  By indicating the higher probability of closer than expected objects, our localization became robust enough to the unmapped obstacles to maintain a heading and lead the Racecar to the proper goal.  

(Does this go in ROS implementation?)
The main ROS node active in our algorithm is named /obstacle_avoidance.  This node takes in localization data as a PoseStamped message from the topic /pf/viz/inferred_pose and laser scan data from the topic /scan.  It then outputs data as an Ackermann message to the topic /vesc/high_level/ackermann_cmd_mux/input/nav_0.  While these topics envelop the technical functionality of our system, we also utilize the topics /heading and /safety as visualization tools.  /heading displays the optimal heading determined by our algorithm as a cone of the angle defined as our section size and length of our clearance distance threshold.  Similarly, /safety is a cone displaying the angle, direction, and threshold of our safety section.  whenever possible.  

## 3) Comparison of Strategies

## 4) Parameter Tuning

### 4.1) Particle Filter Tuning

 - p_short goes up

### 4.2) Algorithm Tuning

 - Many variables to tune: Ackermann steering parameters, clearance thresholds, sections sweep parameters, velocity controls, acceleration limits, gains on controller
 - Wanted something that would be robust enough to work on most possible courses
 - Took a lot of time, eventually was able to reach consistent performance 
 - Key was to lower gain on keeping a straight path so that car was willing to turn more aggressively and to lower its max acceleration so that it wouldn't speed ahead and fail to slow down in time

## 5) ROS Implementation 

## 6) Experimental Results

## 7) Future Work