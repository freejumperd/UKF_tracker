# Unscented Kalman Filter Project

In this project, Unscented Kalman Filter (UKF) is implemented to estimate the state of multiple cars on a highway using noisy lidar and radar measurements.

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The traffic cars will be accelerating and altering their steering to change lanes. The `Z` axis is not taken into account for tracking, so you are only tracking along the `X/Y` axis.

Each of the traffic car's has its own UKF object generated for it, and will update each individual one during every time step using Constant Turn Rate and Velocity (CTRV) motion model.
![image](https://user-images.githubusercontent.com/31724244/180667206-e7383b04-f3f4-4bc7-a971-b04782598d01.png)
![image](https://user-images.githubusercontent.com/31724244/180667210-65906e54-f31f-4047-89e8-2f6a4531f1eb.png)


The accuracy will be evaluated by the Root Mean Squared Error (RMSE) over each time step and for each car.

## I. System Preparations

#### Build requirements

The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ukf_highway


## II. Debugging Details

In the `highway.h`, there are a number of parameters we can modify for debugging purpose.

- `trackCars` list can toggle on/off cars for UKF object to track
- `projectedTime` and `projectedSteps` controls the visualization of predicted position in the future
- `visualize_pcd` sets the visualization of Lidar point cloud data

```c++
// Set which cars to track with UKF
std::vector<bool> trackCars = {true,true,true};
// Visualize sensor measurements
bool visualize_lidar = true;
bool visualize_radar = true;
bool visualize_pcd = false;
// Predict path in the future using UKF
double projectedTime = 0;
int projectedSteps = 0;
```

## III. Code Walkthrough

1. Initialize the UKF attributes 

- dimension of the state vector 
- state vector 
- covariance matrix 
- dimension of the augmented state vector
- predicted sigma points matrix 
- sigma points weights vector 
- standard deviation of longitudinal acceleration noise 
- standard deviation of yaw acceleration noise 
- sigma points spreading parameter 

2. Implement UKF::ProcessMeasurement()
![image](https://user-images.githubusercontent.com/31724244/180667224-88011a73-254b-4f03-bccb-5f2653f672f8.png)

For the very first incoming measurement, state vector `x_`, covariance matrix `P_`, and timestamp `time_us_` are initialized according to the raw data `meas_package.raw_measurements_` and `meas_package.timestamp_`.

For the following measurements, timestamp `time_us_` is recorded, a sequence of functions are called to `Prediction()` and `UpdateLidar()`/`UpdateRadar()`.

3. Implement UKF::Prediction()
Prediction
Generate Sigma Points
![image](https://user-images.githubusercontent.com/31724244/180667247-94cf9c49-7ca4-4616-871a-a5f0d8c11f69.png)
![image](https://user-images.githubusercontent.com/31724244/180667254-f05542ec-8aa3-4c93-8e84-4fb7ae2716bc.png)
![image](https://user-images.githubusercontent.com/31724244/180667261-24f754b0-258c-4e6e-8097-3f13c2992528.png)

The prediction process is the same for both Lidar and Radar measurements.

- creates an augmented mean vector and augmented state covariance matrix
- generate sigma points matrix for previously estimated state vector
- predict sigma points matrix for the current state vector 
- predict the state mean and covariance using weights and predicted sigma points

4. Implement UKF::UpdateLidar()` and `UKF::UpdateRadar() 
The steps to update Lidar and Radar measurements are similar.
However the Lidar sensor is a linear model using x,y position in points Cartesian coordinates while Radar sensor is a nonlinear model measuring bearing, range, bearing rate in Polar coordinates. Therefore, they differ in the measurement dimension dimension of matrices, and the transformation equations.

UKF Update
![image](https://user-images.githubusercontent.com/31724244/180667370-206ef277-686e-4396-9d67-68a141bd8f74.png)

As learned from above, the general UKF measurement update follows the process: 
- transform the predicted sigma points into measurement space based on the sensor types
- calculate the mean state and covariance matrix with noise considered
- calculate cross-correlation matrix between state space and measurement space
- calculate the Kalman gain 
- update the state vector and covariance 

5. Test 
As shown in the screenshot below. The ego car is green while the other traffic cars are blue. The red spheres above cars represent the `(x,y)` lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The green spheres above cars represent the predicted path that cars would move in the near future.

On the left-hand side, the root mean squared errors (RMSE) for position `(x,y)` and velocity `(Vx, Vy)` are calculated in realtime, which represent the prediction accuracy.
![image](https://user-images.githubusercontent.com/31724244/180667621-21af4e3a-e2e8-4ae4-9439-448af6f92992.png)

6. Analysis and tunning parameters:

In order to test the UKF's convergence performance. Different initial values for 
the state vector, 
covariance matrix, 
standard deviation of longitudinal acceleration noise,
standard deviation of yaw acceleration noise
Are used to test with 0 (unknown initial states) and the below refined parameters. 
The results shown that UKF is capable to robustly and quickly converge state estimation of all three vehicles. 

And the following parameters are used to provide he RMSE values are always within the thresholds during the simulation.
std_a_ = 1.0;
std_yawdd_ = 1.0

/* For Lidar */
![image](https://user-images.githubusercontent.com/31724244/180667438-08b1e268-c9e0-4751-b74b-62d26ab2a124.png)


/* For Radar */
![image](https://user-images.githubusercontent.com/31724244/180667452-4277ce91-4d89-4924-9503-df9ca92b434c.png)

Video:
https://user-images.githubusercontent.com/31724244/180667587-c741743f-6459-4519-9725-65f6c633513e.mp4

