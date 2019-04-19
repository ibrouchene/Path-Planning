# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## My implementation
### Path planner
The path planning code is for simplification purposes self contained in the src/main.cpp file. As shown during the course, it basically consist of three steps:
1- Situation analysis: observe the surrounding traffic information, given by the sensor fusion, and predict the trajectories of the other vehicles
2- Behavior planning: based on the situation analysis, decide what is the best thing for the ego vehicle to do.
3- Trajectory generation: generate a smooth trajectory as a result of the behavior planning.

### Situation analysis
In this block we need to go through the object list from the sensor fusion and find out if our path is blocked, and if the neighbouring lanes are also blocked, not allowing us to change lanes (line block 279-345). After many trials and several accidents due to unsafe overtaking maneuvers, I decided to consider a lane change safe if a certain minimal distance was there between the ego vehicle and the target vehicle on the neighbouring lanes. A lane change is possible if the car on the next lane is at least at 20m, or 15m but slower than us. If there is a car under 30m ahead of us on our lane, we shall consider slowing down.

### Behavior planning
We want to drive optimally around the track. This means that if we approach a vehicle that is slower than the speed limit, we should try to overtake it or slow down if a safe overtaking is not possible. This is done in the code block between lines 347 and 378. The define ACCEL_DELTA_MAX is set to such a value that jerk and acceleration stay in acceptable range. A minimal speed of 5mph is set so as to never have to completely stop on the freeway. This is just a safety for this project, in a production SW this should not be in there here.

### Trajectory generation
The line block starts at 378. The suggested spline implementation is used for this. As suggested in the course, the last two points of the previous trajectory are used when possible in order to ensure the smoothest possible trajectory. The decided target speed is used in order to sample the trajectory, ensuring that the speed is respected.


## Running the SW
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

### Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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

