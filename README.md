# **Path Planning Project**

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

[//]: # "Image References"

[image1]: ./examples/undistort_output.png "Undistorted"

## [Rubric](https://review.udacity.com/#!/rubrics/1971/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Compilation

#### 1. The code compiles correctly.  

You're reading it!

### Valid Trajectories

#### 1. The car is able to drive at least 4.32 miles without incident.

The code for this step is contained in the first code cell of the IPython notebook located in "./P2.ipynb".  

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

#### 2. The car drives according to the speed limit.

#### 3. Max Acceleration and Jerk are not Exceeded.

#### 4. Car does not have collisions.

#### 5. The car stays in its lane, except for the time between changing lanes.

#### 6. The car is able to change lanes

### Reflection

#### 1. There is a reflection on how to generate paths.

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image2]

In order to do it the points computed during the camera calibration step have been used to undistort the image through the `cv2.undistort()` function obtaining the following result:

![alt text][image7]
