# **Path Planning** 
---

**Path Planning Project**

The goals / steps of this project are the following:
* Process the data from the sensor fusion module and estimate the road condition(Whether the lanes are occupied by other vehicles).
* Build and state machine to decide what maneuver the vehicle should take(Lane keep, lane change).
* Build the path planning algorithm corresponding to each maneuver.
* Compile the code and test the program on the simulator.
* Summarize the results with a written report

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) individually and describe how I addressed each point in my implementation.  

---
### Compilation

#### 1. The code compiles correctly.

My project includes the following files:
* main.cpp containing the script to process the data from sensor fusion, state machine, and path-planning algorithm.
* helpers.h containing all the helper functions
* spline.h containing the spline library used for the path planning algorithm
* Model Documentation.md summarizing the results

The code could be compiled correctly under the `CarND-Path-Planning-Project/build` folder running `cmake .. && make`. The path_planning file will be generated.

Using the Udacity provided simulator and running the `path_planning` file under build folder, the path is planned autonomously on the highway scenario.

### Valid Trajectories

#### 1. The car is able to drive at least 4.32 miles without incident.

This point is check by running the algorithm in the simulator.

#### 2. The car drives according to the speed limit.

In the file main.cpp line 205 the speed limit is set to be 75km/h, which is to guarantee the maximum speed is always under the speed limit.

#### 3. Max Acceleration and Jerk are not Exceeded.

In the file main.cpp line 200 and 205. The incremental step of the velocity and decceleration and acceleration are set to be -0.5 and 1.2 m/s, which turned out to be proper values to avoid the max acceleration violating the limit.

In the file main.cpp line 210. The first anchor point to define the spline is set to be 47m ahead of the current ego vehicle position in the s coordinate. By running the simulation it turned out to be a proper distance to avoid the jerk to exceed to limit.

#### 4. Car does not have collisions.

The state machines are designed to avoid collision with other vehicles. It will be described in details later.

#### 5. The car stays in its lane, except for the time between changing lanes.

In the file main.cpp in the lines 209-211 the anchor points are derived in the Frenet coordinate and then transform into XY coordinate. It could guarantee the ego vehicle always travel in its lane.

#### 6. The car is able to change lanes.

The state machine is designed to guarantee that the car could change lanes when a slow vehicle is in the front and the side lanes are free.

### Reflection

#### 1.There is a reflection on how to generate paths.

In this file Model Documentation the algorithm of path generation will be described later.

## Path Planning algorithm

### Deriving the first two anchor points and local reference system

Lines 117 - 137 derive the first two ref points and the ref yaw in the anchor points set, which is to guarantee that the following planned path is smooth. The coordinate of the anchor point is rotated by an angle of the car's yaw angle and shifted to the last point(the reference point) in the previous planned points.

### Verifying the road condition

In this section(lines 140 - 184) all the vehicle information in sensor_fusion are traversed. Generally the outputs contain two boolean variables.If there is a vehicle presented in front and in the same lane of the ego vehicle, and the gap is under the threshold,the front_occupied flag will be set to true. And also if the side lanes are available for lane changing are checked. And store in `lane_occupied`.`lane_occupied[0]`,`lane_occupied[1]`,`lane_occupied[2]` represent the lanes from left to right respectively.

### State transition

In this section(lines 187 - 207) the state machine is designed in the following section.If the ego vehicle is obstructed by a slow vehicle in the front, the availability of lane changing to the adjacent lane will be check. If the adjacent lane is free, then the  lane variable will be set to the number of the adjecent lane. Else if all the adjacent lanes are not safe for lane change, the ego vehicle will keep it lane and slow down its speed.

### Determining the spline function

In lines 210 - 237. Once the target lane is obtained, the next three anchor points are define 40, 80, and 110 meters ahead of the ego vehicle in the s coordinate.The coordinates in the global reference system are obtained using the getXY function. The three new anchor points are also rotate and translated in the local reference system of the last point of the previous planned path. Finally the spline function is created with the five anchor points.

### Generating the points in the path using the spline function

Lines 240 - 257. Find out the points to be added into the planning point set by interpolating the using the created spline function. The step of the longitudinal direction is proportional to the target speed. The corresponding y with respect to each step x is derived using the spline function in the local reference frame.For each point added in the local reference frame, it will be transformed back into the global reference frame. Finally it will be added into the planned point list.

