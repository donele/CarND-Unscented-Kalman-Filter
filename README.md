# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

An [Unscented Kalman Filter](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf) is impelmented to estimate the state of a moving object.
A constant turn rate and velocity magnitude model (CTRV) is used to represent the state.
Measurements from a radar and a lidar are combined.

## Tuning Process Noise

The process noise `std_a_` and `std_yawdd_` are free parameters that need to be tuned.
`std_a_` is the acceleration noise in the direction of the object's trajectory.
According to a [study](https://www.pdx.edu/ibpi/sites/www.pdx.edu.ibpi/files/Bicycle%20Performance%20Forthcomming%202013.pdf), the average speed and acceleration of a bicycle in a city are 5 m/s and 0.5 m/s^2. I chose the half of the value, 0.25 m/s^2, to set the parameter `std_a_`.

`std_yawdd_` is the noise that changes the yaw angle. In order to estimate its magnitude, I imagined a bike making a 90 degree turn along a circle with radius 5.55 meters. That is 1.5 times 3.7 meters, which is a typical lane width in the US. A bike would move on such a trajectory if it makes a left turn from the center of a right lane. With a speed of 5 m/s, the angular acceleration will be 0.9 rad/s^2. I take half of the value, 0.45, to set the parameter `std_yawdd_`.

## First measurement

The first measurement is used to set the location values p_x and p_y of the state vector. The other values, v, psi, and psi_dot are set to zeros.

## Result


The RMSE of the kalman filters for the first data set is as following.

|           |px       |py      |vx      |vy      |
|:---------:|:-------:|:------:|:------:|:------:|
|Laser+Radar|0.0600   |0.0937  |0.3847  |0.2600  |
|Laser only |0.1648   |0.1057  |0.5223  |0.3285  |
|Radar only |0.2111   |0.2597  |0.4648  |0.3088  |































