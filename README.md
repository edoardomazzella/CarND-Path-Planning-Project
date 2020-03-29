# **Path Planning Project**

## Goals

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Soulution Description

The `MotionPlanner` class offers the public method `GenerateTrajectory` in order to iteratively compute a path to follow given the information about the car localization, the map and the sensor fusion data.
The `GenerateTrajectory` algorithm is composed by several steps and substeps briefly described below:
* 1 `PlanBehavior()` plans the car velocity and the future lane:
    * 1.1 Assigns each percepted car to a lane.
    * 1.2 `UpdateTooClose_()` checks if there is a vehicle in the same lane that is too close.
    * 1.3 `AdaptVelocity_()` adapts the car velocity if there is a vehicle that is too close.
    * 1.4 `UpdateLane_()` updates the selected lane based on the cost functions `KeepLaneCost_()` and `ChangeLaneCost_()`, the choice must be stable over 10 time steps to activate a lane change action.
        * 1.4.1 `KeepLaneCost_()` takes as input the main car information and the information about the other cars in the lane (output of step 1.1) and increments the cost of staying in the lane proportionally to the difference of speed with the vehicle in front of us.
        * 1.4.2 `ChangeLaneCost_()` takes as input the main car information and the information about the other cars in the lane (output of step 1.1) and depending on the distance of the other vehicles and their velocity it increments or decrements the cost of changing lane. The cost is modified differently if the vehicle is ahead of us or behind of us.
* 2 Trajectory Planning:
    * Trajectory planning algorithm is the one already described in the project lesson `Project Q&A`. It generates a list of N points spread over a chosen number of meters. After taking a set of reference points from the previous path and computing evenly spaced points ahead of the starting reference, these points are interpolated using a spline based on the chosen number of points and meters.

Given this information the code is self-descriptive

## [Rubric](https://review.udacity.com/#!/rubrics/1971/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Compilation

#### 1. The code compiles correctly.  

Code must compile without errors with cmake and make.

`CMakeLists.txt` has been modified to properly compile the project.

### Valid Trajectories

#### 1. The car is able to drive at least 4.32 miles without incident.

This requirement is largely satisfied.

#### 2. The car drives according to the speed limit.

The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.

#### 3. Max Acceleration and Jerk are not Exceeded.

The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

#### 4. Car does not have collisions.

The car does not come into contact with any of the other cars on the road.

#### 5. The car stays in its lane, except for the time between changing lanes.

The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

#### 6. The car is able to change lanes

The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

### Reflection

#### 1. There is a reflection on how to generate paths.

You can read it above.
