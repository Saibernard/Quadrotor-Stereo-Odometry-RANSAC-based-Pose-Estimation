# Quadrotor-Stereo-Odometry-RANSAC-based-Pose-Estimation
This repository presents a robust method for estimating the pose of a quadrotor using stereo correspondences. By leveraging the RANSAC algorithm and least squares techniques, the code offers a reliable way to analyze real data from the EuRoc dataset, ensuring accurate quadrotor pose predictions.

## Overview

### 1. Introduction
This project delved into pose estimation of a quadrotor using stereo correspondences. By implementing the algorithmic approaches discussed in prior sessions, we applied our methods to the EuRoc dataset from ETH Zurich. The dataset is comprehensive, comprising of both images and IMU data. A paper associated with the assignment provides details about this dataset.

## 2. Code Architecture
The provided code structure encompasses the following key components:
- **Utility Scripts**: These include tests, allowing for validation of the code.
- **Dataset Segment**: Contains stereo and IMU data from the EuRoc dataset.
- **Core Coding Files**: These are essential for the project. Of note, `stereo.py` is responsible for reading from the dataset, and `estimate_pose_ransac.py` holds the core for implementing the RANSAC-based stereo odometry.

## 3. Core Implementation Details:

### Task 1: Least Squares Solve
To compute the quadrotor's pose, a least-squares method was utilized. Given input arrays uvd1 and uvd2, which are 3×n dimensional and represent normalized stereo measurements in two frames, the task was to estimate rotation (w) and translation (t).

Given:
<div style="font-size: 1.5em;">
u′ = X/Z
v′ = Y/Z
d′ = 1/Z
</div>
Using the numpy function `lstsq`, a least squares estimate was computed for this linear system to derive w and t.

### Task 2: Finding Inliers
With the primary function estimating rotation and translation, outliers can skew the results. To address this, the RANSAC procedure identifies inliers based on a discrepancy vector, δ, for each correspondence.

Inliers were recognized by:

![image](https://github.com/Saibernard/Quadrotor-Stereo-Odometry-RANSAC-based-Pose-Estimation/assets/112599512/9047f262-7128-42b3-ad14-48123acd992b)


### Task 3: RANSAC Implementation
The `ransac_pose` function executes the RANSAC algorithm on sensor data, iterating based on the parameter `ransac_iterations`. For the case when `ransac_iterations` is zero, all correspondences are deemed inliers. This function returns the vectors w and t, along with a boolean array signifying inliers for the final solution.

![stereo pair 1](https://github.com/Saibernard/Quadrotor-Stereo-Odometry-RANSAC-based-Pose-Estimation/assets/112599512/6e6fd98c-5bab-40c6-b802-9ea0d0153c11)

![stereo pair 2](https://github.com/Saibernard/Quadrotor-Stereo-Odometry-RANSAC-based-Pose-Estimation/assets/112599512/99bff780-e091-46bd-9aaa-a665e71dd092)

![stereo pair unrectified](https://github.com/Saibernard/Quadrotor-Stereo-Odometry-RANSAC-based-Pose-Estimation/assets/112599512/ce719d76-beff-45e5-a37c-6af339e41a13)

![temporal matches](https://github.com/Saibernard/Quadrotor-Stereo-Odometry-RANSAC-based-Pose-Estimation/assets/112599512/8fd68d04-e296-4c5e-ab58-673a4ea5dade)


## Conclusion
This project exemplifies the application of RANSAC and least squares methodologies to stereo odometry for quadrotor pose estimation. By carefully implementing these techniques and testing with the EuRoc dataset, a robust and reliable pose estimation method was achieved.


