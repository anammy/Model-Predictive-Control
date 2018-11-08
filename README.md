# Model Predictive Control (MPC)
Self-Driving Car Engineer Nanodegree Program

---

## Objective

The purpose of this project was to build a MPC controller to successfully drive the car around the track. 


[//]: # (Image References)

[image1]: ./pictures/kinematic-eqns.png "KinematicEqns"
[image2]: ./pictures/cte.png "CrossTrackError"
[image3]: ./pictures/epsi.png "OrientationError"
[image4]: ./pictures/cost.png "Cost"

## Controller Design

Model predictive control requires the prediction of trajectories based on sets of steering and throttle actuations over a specified time horizon. This was accomplished through the following kinematic model:

![alt text][image1]

For each predicted trajectory, the associated cost was calculated using the following equations:

![alt text][image4]

The cost consists of the summation of the squared cross-track error (cte), orientation error (epsi), difference in the velocity and the reference velocity, steering angle, difference in steering angle between time steps, acceleration, difference in acceleration between time steps, and the product of the velocity and the steering angle. The different cost components were multiplied by various weights that were tuned manually inorder to successfully drive the car around the track and provide a smooth ride. The cost was minimized with respect to actuations such as steering angle and throttle. The steering angle was limited to be between -25 deg to 25 deg and the throttle was limited to be between -1 (full brake) to 1 (full acceleration). The predicted trajectory that minimized the cost was chosen as the correct path to follow and only the first set of actuations were performed. This procedure was then repeated for the next time step. The predicted trajectories were calculated for a time horizon of 1 sec with a timestep length N = 10 and the elapsed duration between timesteps dt = 0.1. The predicted trajectories for shorter time horizons (i.e. 0.7 sec, 0.6 sec, and 0.5 sec) weren't able to compensate for sharp turns very well. The predicted trajectories for longer time horizons (i.e. 1.5 sec) were unable to properly track the reference trajectory at the furthest points. As a result, it was wasted computation with no gain in useful information.

The cross track and orientation errors were calculated by fitting the waypoints from the reference trajectory with a 3rd degree polynomial and taking the difference between the predicted trajectory from the kinematic equations and this fitted polynomial. The waypoints provided from the simulator were in map/global coordinates and thus, were transformed into vehicle coordinates through a coordinate system rotation prior to fitting them with a polynomial. The cross track and orientation errors were calculated using the following equations:

![alt text][image2]
![alt text][image3]

There was an actuation latency of 100 ms that was modelled into the controller. The vehicle state after 100 ms was calculated from the current state given by the simulator through the use of the kinematic equations. The predicted vehicle state after the 100 ms delay was then sent to the MPC solver to calculate the next set of actuations to be executed.

## Code

The default Udacity code directory structure and starter code were used. The files `main.cpp`, `MPC.h`, and `MPC.cpp` were editted to implement the MPC controller and for cost factor tuning.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Result

A video of the car successfully driving around the track is provided [here](https://github.com/anammy/Model-Predictive-Control/tree/master/video).
