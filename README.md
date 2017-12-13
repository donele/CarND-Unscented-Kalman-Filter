# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

An [Unscented Kalman Filter](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf) is impelmented to estimate the state of a moving object. In the previous project, I have put together a set of classes that separates the implementaion specifics of two sensors. I am using a similar structure in this project, rather than using the provided file structure.

A class `FusionUKF` is created in the `main` function. The kalman filter algorithms are implemented in the classes `UKFRadar` and `UKFLaser`. The two classes inherits from `UKF` that implements some common functionalities.

A constant turn rate and velocity magnitude model (CTRV) is used to represent the state. The state is imeplemented in the class `StateCTRV`. An instance of `StateCTRV` is created as a private member of `FusionUKF`. The reference to the state is passed to the member functions of `UKF` and its subclasses to be updated.

## Tuning Process Noise

The process noise `std_a_` and `std_yawdd_` are free parameters that need to be tuned.
`std_a_` is the acceleration noise in the direction of the object's trajectory.
According to a [study](https://www.pdx.edu/ibpi/sites/www.pdx.edu.ibpi/files/Bicycle%20Performance%20Forthcomming%202013.pdf), the average speed and acceleration of a bicycle in a city are 5 m/s and 0.5 m/s^2. I chose the half of the value, 0.25 m/s^2, to set the parameter `std_a_`.

`std_yawdd_` is the noise that changes the yaw angle. In order to estimate its magnitude, I imagined a bike making a 90 degree turn along a circle with radius 5.55 meters. That is 1.5 times 3.7 meters, which is a typical lane width in the US. A bike would move on such a trajectory if it makes a left turn from the center of a right lane. With a speed of 5 m/s, the angular acceleration will be 0.9 rad/s^2. I take half of the value, 0.45, to set the parameter `std_yawdd_`.

## First measurement

The first measurement is used to set the location values p_x and p_y of the state vector. The other values, v, psi, and psi_dot are set to zeros.

The covariance matrix `P` is initialized as a diagonal matrix. `P_11` and `P_22` are the uncertainties for the locations. I set the values as the standard deviations of lidar measurements, which is 0.15. The third diagonal element `P_33` is the uncertainty for the velocity. I set the value as half of the average speed of a bicycle quoted above, 2.5 m/s. The fourth element is the uncertainty of the yaw angle. The full range of the angle is `2 * pi`, roughly 6.3. I take about a quarter of the value, 1.5, as its initial value. The final element is the uncertainty of the change rate of yaw angle. I use the same value as `std_yawdd_`, 0.45.

## Result


The RMSE of the kalman filters for the first data set is as following.

|           |px       |py      |vx      |vy      |
|:---------:|:-------:|:------:|:------:|:------:|
|Laser+Radar|0.0599   |0.0939  |0.3789  |0.2409  |
|Laser only |0.1588   |0.1475  |0.3991  |0.2410  |
|Radar only |0.2097   |0.2164  |0.4502  |0.2293  |

When only one of the sensors is used, laser does better job at determining the location, as expected.

For comparison, below is the result from the extended kalman filter.

|           |px       |py      |vx      |vy      |
|:---------:|:-------:|:------:|:------:|:------:|
|Laser+Radar|0.0983   |0.0852  |0.4071  |0.4682  |

The unscented kalman filter redueces the location error by about 16%, and the speed error by 29%, compared to the extended kalman filter.































