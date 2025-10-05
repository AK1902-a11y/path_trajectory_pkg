# <u>Autonomous Navigation System for Differential Drive Robots</u>
A comprehensive ROS2-based navigation stack featuring advanced path smoothing, trajectory generation, and tracking controllers for differential drive mobile robots.

## <u>Project Overview</u>

This project implements a complete autonomous navigation pipeline for differential drive robots, transforming discrete waypoints into smooth, dynamically-feasible trajectories with real-time tracking control. The system demonstrates two distinct implementation approaches, each optimized for different performance characteristics.

### Core Capabilities:

- Smooth path generation from discrete waypoints

- Time-optimal trajectory planning with kinematic constraints

- Real-time trajectory tracking with velocity regulation

- Performance monitoring and visualization

 ### Test Environment:

- Robot Platform: BCR Bot (Differential Drive)

- Simulation: Gazebo Fortress

- Framework: ROS 2 Humble

## <u>System Architecture</u> 

Both implementations follow a three-stage pipeline:

Waypoints → Path Smoothing → Trajectory Generation → Controller → Robot Motion

### Implementation 1: Baseline System
- Path Smoother: B-Spline Interpolation

- Trajectory Generator: Curvature-Aware Forward-Backward Pass

- Controller: Pure Pursuit

### Implementation 2: Enhanced System
- Path Smoother: Clothoid Curve Interpolation

- Trajectory Generator: Curvature-Aware Forward-Backward Pass

- Controller: Regulated Pure Pursuit

## <u> Algorithm Selection & Implementation Approach</u>

### Development Methodolgy
This project follows a systematic evaluation and iterative refinement approach, where each algorithmic component was selected through comparative analysis, empirical testing, and performance validation. The navigation system was developed for a differential drive robot, requiring smooth, collision-free path execution.

### Phase 1: Path Smoothing Algorithm Evaluation

#### Test Paths:

- Square Path: 4 waypoints with sharp 90° corners - stress test for algorithm robustness

- S-Curve Path: 4 waypoints with natural curved transitions - typical warehouse maneuvers

Sampling Density: 500 interpolated points per path for high-resolution analysis

#### Candidate Algorithms

- B-Spline Interpolation (scipy.interpolate.splprep) with s=0

- Cubic Spline Interpolation (scipy.interpolate.CubicSpline)

- Clothoid Curves (pyclothoids library) (Reserved for Implementation 2)

#### Comparative Analysis: B-Spline vs Cubic Spline

##### <u>>Test 1: Square Path (Sharp Corner Performance)</u>

|Metric           |B-Spline|CubicSpline|Difference|Significance              |
|-----------------|--------|-----------|----------|--------------------------|
|Original Length  |8.00 m  |8.00 m     |-         |Baseline                  |
|Smooth Length    |9.03 m  |8.52 m     |+6%       |B-Spline slightly longer  |
|Max Curvature    |1.19 m⁻¹|1.32 m⁻¹   |-11%      |B-Spline safer           |
|Avg Curvature    |0.72 m⁻¹|0.64 m⁻¹   |+12%      |B-Spline more conservative|
|Smoothness (∑Δκ²)|0.050   |0.080      |-60%      |B-Spline superior        |
|Computation Time |0.0006 s|0.0006 s   |Equal     |Both real-time capable    |

##### <u>>Test 2: S Curved Path(Sharp Corner Performance)</u>

|Metric           |B-Spline|CubicSpline|Difference|Significance              |
|-----------------|--------|-----------|----------|-------------------------- |
|Original Length  |3.828 m | 3828. m   | -        | Baseline                  |
|Smooth Length    |3.969 m | 3.895 m   | +2%      | Negligible Difference     |
|Max Curvature    |0.72 m⁻¹| 0.86 m⁻¹  | -19%     | B-Spline safer            |
|Avg Curvature    |0.65 m⁻¹| 0.50 m⁻¹  | +30%     | Tradeoff Smoothness       |
|Smoothness (∑Δκ²)|0.0006  | 0.0097    | -1500%   | B-SplineExceptional       |
|Computation Time |0.0006 s| 0.0006 s  | Equal    | Both real-time capable    |

# Phase 2: Clothoid Curves for Enhanced Implementation
While B-splines provide excellent performance, clothoid curves offer theoretical optimality for differential drive robots based on time-optimal control theory. Research demonstrates 15-25% execution time improvements due to natural kinematic compatibility.

### Phase 3: Trajectory Generation Strategy

#### Candidate Algorithms Considered
Trajectory generation algorithms:

- Trapezoidal Velocity Profile

- Curvature-Aware Forward-Backward Pass

- Minimum Jerk Trajectory

- Polynomial-Based Trajectories

- Time-Optimal Planning (Pontryagin's Principle)

- Spline-Based Velocity Planning

#### Why Curvature-Aware Forward-Backward Pass:

This algorithm was selected because it integrates path geometry directly into velocity planning. The key advantage is that it automatically adjusts velocity based on local path curvature, preventing overspeed on curves while maximizing velocity on straight sections. This ensures:

- Safe navigation through curved sections without wheel slip

- Time-optimal execution within kinematic constraints

- Smooth, continuous velocity profiles

#### Why NOT Trapezoidal Velocity Profile:

Trapezoidal profiles were rejected because they assume constant velocity along the entire path, ignoring curvature variations. This creates critical safety issues:

- No curvature awareness: Robot maintains high velocity through sharp turns, risking wheel slip

- Controller stress: Pure Pursuit must compensate for excessive speeds on curves, causing tracking errors

- Inefficient: Either overly conservative (slow everywhere) or unsafe (fast on curves)

- Poor trajectory quality: Cannot balance speed on straights with safety on curves

The curvature-aware approach provides automatic velocity modulation that trapezoidal profiles cannot achieve, making it essential for differential drive robots navigating paths with varying geometry.

### Phase 4: Data Pipeline Architecture

#### YAML-Based Modular Design Philosophy

Design Rationale: Decouple navigation modules for independent development, testing, and debugging

Key Benefits:

- Independent Module Testing: Each stage validated before integration

- Human-Readable Formats: Easy debugging with text editors

- Version Control Friendly: Git tracks changes effectively

- Incremental Development: Add features without breaking pipeline

Different yaml files created are stored in Config directory in root of ros2 workspace automatically and loaded from there, different yaml files used are:

path_smoother -> smooth_path.yaml -> trajectory generator -> trajectory.yaml -> trajectory_controller

### Phase 5: Visualization & Debugging

#### Automated Visual Validation
Each pipeline stage generates diagnostic plots for visual validation before robot deployment:

#### Path Smoother Outputs:

- Waypoints overlaid with smoothed path

- Curvature profile vs arc length

- Comparison metrics table (when applicable)

#### Trajectory Generator Outputs:

- Velocity profile vs time/distance

- Acceleration profile verification

- Path with velocity-based color coding

### Phase 6 Test Files

There are 6 test files present inside the test directory, these are jupyter notebook test for each of the 6 scripts and these tests files are;

- test_path_smoother_bspline.ipynb for testing path smoother with b spline algorithm

- test_path_smoother_clothoids.ipynb for testing path smoother with clothoid algorithm

- test_trajectory_generator.ipynb for testing trajectory generation using curve awareness algorithm

- test_trajectory_controller.ipynb for testing pure pursuit controller

- test_trajectory_controller_rpp.ipynb for testing  regulated pure pursuit controller


Build workspace before running test scripts

##  <u>Setup and Execution</u>

###  Setup 

#### Software Requirements

- ROS 2 Distribution: Humble

- Python: 3.8 or higher

- Gazebo: Fortress or Classic 11

- matplotlib, numpy, scipy

- jupyter notebook

- pyclothoid library

``
pip3 install pyclothoids
``

- Transformations 

``
pip3 install transforms3d
``

### To test the Simulation it is required to build the bcr_bot and test if it is running [https://github.com/blackcoffeerobotics/bcr_bot]

###  Note - I'm running everything inside a docker container, which is not necessary.

### Execution 
Each of the scripts except path_smoother contains help text and cli arguments, more information can be known using --help flag when executing the script, for eg:

``
ros2 run path_trajectory_pkg trajectory_controller --help
``

this contains all cli arguments that can be used and passed while running the script.

### The directory structure is:

path_trajectory_pkg

|

path_trajectory_pkg -> scripts

|

launch -> launch files

|

tests -> jupyter notebook tests files

This package contains 2 launch files 

- implementation1_launch.py - This launches path_smoother and trajectory generator script with default arguments for implementation 1 , i.e path smoother is bspline 

- implementation2_launch.py - This launches path_smoother and trajectory generator for implementation 2, i.e path smoother is clothoid

Script 3 which is controller script needs to be executed after running the gazebo simulation 

All the scripts can be run indivdually with different parameters for testing purpose.

### Executing Commands

#### Implementation 1

- Run the implementation1 launch file 

``
ros2 launch path_trajectory_pkg implementation1_launch.py
``

- Run the gazebo simulation (bcr bot)

``
ros2 launch bcr_bot ign.launch.py
``

This runs in gazebo ignition but gazebo classic can also be used for simulation.

``
ros2 launch bcr_bot gazebo.launch.py
``

- Run controller script 

``
ros2 run path_trajectory_pkg trajectory_controller
``

- Path smoother and trajectory generator script can also be run individually without launch file

``
ros2 run path_trajectory_pkg path_smoother 
``

``
ros2 run path_trajectory_pkg trajectory_generator
``
#### Implementation 2

- Run the implementation2 launch file 

``
ros2 launch path_trajectory_pkg implementation2_launch.py
``

- Run the gazebo simulation (bcr_bot)

for gazebo ignition environment


``
ros2 launch bcr_bot ign.launch.py
``

for gazebo classic environment

``
ros2 launch bcr_bot gazebo.launch.py
``

- Run controller script

``
ros2 run path_trajectory_pkg trajectory_rpp_controller
``

- Path smoother and trajectory generator script can also be run individually without launch file

``
ros2 run path_trajectory_pkg path_smoother_clothoids
``

``
ros2 run path_trajectory_pkg trajectory_generator
``

















