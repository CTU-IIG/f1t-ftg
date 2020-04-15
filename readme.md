# Follow The Gap (CTU)
This repository contains an implementation of Follow The Gap algorithm [1].
It differs slightly from the original, as the proposed algorithm lacked some features to withstand the issues with real (competition) scenarios.

This implementation can be used in two different ways -- as a separate library (by running `make`) or integrated to ROS and compiled by `catkin`.



## Compilation


### ROS version
The repository contains all necessary files to make it compatible with ROS (tested on `ROS Kinetic`). Simply add it to your workspace and compile it using `catkin_make`.

Required packages:
```
roscpp
std_msgs
sensor_msgs
visualization_msgs
geometry_msgs
tf
```


### No-ROS library
The code is split into several files to allow creating a ros-independent library `libftg`. Navigate to the root folder and run `make` to compile the code and create the library. In this case ROS is not required at all.



## Running


### ROS
After compilation, binary `follow_the_gap` is created. Using following command, the node can be run:

```
rosrun follow_the_gap follow_the_gap
```

Upon successful node registration, these topics are created:

| IN / OUT    | Topic name                     | Message type                | Description
| ----------- | ------------------------------ | --------------------------- | -----------
| IN          | /scan                          | sensor_msgs::LaserScan      | Main source of information, received from LiDAR
| IN          | /lsr/angle                     | std_msgs::Float64           | Goal angle
| IN (unused) | /left_constraint_index         | std_msgs::Int32             | Limiter of final angle (left)
| IN (unused) | /right_constraint_index        | std_msgs::Int32             | Limiter of final angle (right)
| OUT         | /final_heading_angle           | std_msgs::Float32           | Final heading angle
| OUT         | /visualize_final_heading_angle | geometry_msgs::PoseStamped  | Final heading angle for rViz
| OUT         | /visualize_obstacles           | visualization_msgs::Marker  | LiDAR measurements used by the used approach for rViz
| OUT         | /visualize_largest_gap         | geometry_msgs::PointStamped | Alternating 3 values containing car position, left and right side of the largest gap for rViz


### Without ROS
The algorithm is run by calling function `FollowTheGap::Callback` with following arguments:
 - `const std::vector<Obstacle> & obstacles` -- array of obstacles,
 - `LidarData * lidar_data` -- information about lidar sweep for filtering,
 - `std::vector<Obstacle> & obstacles_out` -- vector for returning used obstacles, and
 - `std::vector<Obstacle> & gap_borders_out` -- vector for returing the borders of the largest gap.

This function returns `std::tuple<bool, float>` with these values:
 - `bool ok` -- true when the largest gap was identified and other field contains valid data, and
 - `float angle` -- final heading angle.

Note: When the algorithm was not successful, the vector `obstacles_out` should still contain valid data (obstacles used for the last iteration of the algorithm), however the vector `gap_obstacles_out` may be without elements.



## Authors
- Anders Solberg Pedersen
- Jaroslav Klapálek



## References
[1]: V. Sezer and M. Gokasan, ‘A novel obstacle avoidance algorithm: “Follow the Gap Method”’, Robotics and Autonomous Systems, vol. 60, no. 9, pp. 1123–1134, Sep. 2012, doi: 10.1016/j.robot.2012.05.021.



## License
Authors: Anders Solberg Pedersen, Jaroslav Klapálek

Copyright (C) 2020 Czech Technical University in Prague

Repository f1t-ftg contains software follow_the_gap.

follow_the_gap is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.

follow_the_gap is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with follow_the_gap. If not, see <https://www.gnu.org/licenses/>.