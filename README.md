# Robot Control 

## Description

This repository contains code for a angle rate algorithm for a robot using data from a depth camera. The algorithm calculates the angular rate necessary to keep the robot centered in the row, assuming a constant forward velocity.

## 



## Installation
1. Clone the repository:

    ```bash
    git clone https://github.com/vkella/Robot-Control.git
    
    ```

2. Install Dependencies:

    ```bash
    - numpy
    - rospy (Based on your ROS version)
    -opencv-python
	-geometry_msgs (Based on your ROS version)
    -matplotlib
    -math
	-unittest
	
    ```
	
## Tasks 
### Task 1: Robot Angle of Orientation Calculation

The code implements a robot angle calulation algorithm. It processes point cloud data from NPZ files, leveraging computer vision techniques, and outputs the calculated angle in degrees.

```bash
    Task_1/Task_1.py
	
    ```
	Input : NPZ File(Point Cloud Data Stored in Numpy array)
    Output: Angle in Degrees
	
### Task 2: ROS Node 

The code implements a ROS Node which publishes the calculated angular rate from Task 1 as a twist message to the robot's topic cmd_vel.

```bash
    Task_2/Task_2.py
	
    ```

### Task 3: Algorithm Unit Testing

This code implements the basic unit testing of the algorithm developed in Task 1. It includes tests for reading point cloud data, line detection, and angle calculation functions.
 ```bash
    Task_3/Task_3.py
	
    ```
 
 
### Bonus Task: End-of-Row Detection

 A code that contains the logic for detecting the end of a row and update the end flag accordinly.
 
Integration with Task 1 : The end-of-row detection logic can be seamlessly integrated into algorithm with addtion of few lines. 

```bash
    Bonus/Bonus.py
	
    ```

## Contributors
- Vamsee Krishna Kella
	
