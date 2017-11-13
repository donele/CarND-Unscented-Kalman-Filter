# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

A sensor fusion is implemented to estimate the state of a moving object with lidar and radar measurements. A linear kalman filter handles the lidar inputs, and an extended kalman filter is used for the radar measurement.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

## Result

The RMSE of the kalman filters for the first data set is as following.

|           |px       |py      |vx      |vy      |
|:---------:|:-------:|:------:|:------:|:------:|
|Laser+Radar|0.0983   |0.0852  |0.4071  |0.4682  |

### Using only one sensor

Using only one of two sensors, the RMSE is as following.

|           |px       |py      |vx      |vy      |
|:---------:|:-------:|:------:|:------:|:------:|
|Laser only |0.1845   |0.1542  |0.6554  |0.4786  |
|Radar only |0.2321   |0.3361  |0.5317  |0.7139  |

The laser is doing better job at estimating the locations than the radar.

## Implementation

The fusion of the two sensors is accomplished in the class [FusionLaserRadar](https://github.com/donele/CarND-Extended-Kalman-Filter/blob/master/src/FusionLaserRadar.cpp). In the class, two implementations of kalman filters are instantiated. One is the class [KFLaser](https://github.com/donele/CarND-Extended-Kalman-Filter/blob/master/src/KFLaser.cpp), which implements the linear kalman filter to be used with the lidar measurement.

The other implementation is the class [KFRadar](https://github.com/donele/CarND-Extended-Kalman-Filter/blob/master/src/KFRadar.cpp). It implements an Extended Kalman Filter in order to accomodate the nonlinearity in the measurement function for radar. Since the radar measurement occurs in the polar coordinate, a conversion must be made into the Cartesian space, and that is a nonlinear process.

### Inheritance

Both `KFLaser` and `KFRadar` are subclasses of the base class [`KF`](https://github.com/donele/CarND-Extended-Kalman-Filter/blob/master/src/KF.cpp). The class `KF` provides a common interface that processes a measurement, `ProcessMeasurement()`.

```c++
void KF::ProcessMeasurement(KFState& state, float dt, const Eigen::VectorXd& z) {
  Predict(state, dt);
  Update(state, dt, z);
}
```

The base class implements the prediction process in the fuction `Predict()`, which is shared by the two subclasses.

```c++
void KF::Predict(KFState& state, float dt) {
  // Do no predict if dt is very small.
  if(dt < 1e-6)
    return;

  // Prediction calculation is shared by all subclasses.
  set_F(dt);
  set_Q(dt);
  state.x = F_ * state.x;
  state.P = F_ * state.P * F_.transpose() + Q_;

  return;
}
```

The function `Predict()` is defined as a pure virtual function in [`KF`](https://github.com/donele/CarND-Extended-Kalman-Filter/blob/master/src/KF.h), and it is up to each subclass to implement the functionality. An update process of the linear kalman filter is implemented in the class `KFLaser` as following.

```c++
void KFLaser::Update(KFState& state, float dt, const VectorXd &z) {
  // Calculate Kalman gain
  MatrixXd K = state.P * H_trans_ * (H_ * state.P * H_trans_ + R_).inverse();

  // Update the state from the measurement.
  state.x = state.x + K * (z - H_ * state.x);
  state.P = state.P - K * H_ * state.P;
  return;
}
```

In `KFLaser`, the measurement matrix `H` is a constant, and it is calculated in the constructor. Its transpose matrix `H_trans_` is also calculated in the constructor.

An extended kalman filter is implemented in the class `KFRadar`.
```c++
void KFRadar::Update(KFState& state, float dt, const VectorXd &z) {
  // Keep the value of theta in [-pi, pi]
  VectorXd y(3);
  y = z - tools.Cartesian2Polar(state.x);
  float theta = y[1];
  while(theta > PI)
    theta -= 2*PI;
  while(theta < -PI)
    theta += 2*PI;
  y[1] = theta;

  // Calculate jacobian
  H_ = tools.CalculateJacobian(state.x);
  H_trans_ = H_.transpose();

  // Calculate Kalman gain
  MatrixXd K = state.P * H_trans_ * (H_ * state.P * H_trans_ + R_).inverse();

  // Update state and covariant matrices
  state.x = state.x + K * y;
  state.P = state.P - K * H_ * state.P;
  return;
}
```

The measurement matrix `H_` in this case is not a constant, but rather a jacobian that is recalculated each time a new measurement comes in.
 
### Data Encapsulation

The starter code comes with just one kalman filter class that handles both lidar and radar measurement, and many of the kalman filter calculations are done in the class [FusionEKF](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/blob/master/src/FusionEKF.cpp). However, I decided to move all the calculations into the classes `KF` and its subclasses for better data encapsulation. And by using the inheritance between the kalman filter classes, I can reuse the code for the prediction process which is common between two types of kalman filters. A potential risk with this design is that if an algorithm with a different prediction method is implemented in the future, the inheritance structure may have to be rewritten.

The state vector `x` and the covariant matrix `P` are wrapped into a class `KFState`, and it is passed to the functions `Predict()` and `Update()` by reference, to be updated inside the functions.

```c++
class KFState {
public:
  Eigen::VectorXd x;
  Eigen::MatrixXd P;

  KFState();
  virtual ~KFState();
};
```

Only `x` and `P` are accessible from the class `FusionLaserRadar`, and all other covariant matrices are internal to `KF` and its subclasses. Those matrices are only meaningful during the calculations to update `x` and `P`.

### Efficiency

I have tried to avoid evaluating temporary Eigen matrices, so that the compiler can have more oppotunities to optimize the Eigen template operations. More information on the efficient matrix evaluation can be found [here](http://eigen.tuxfamily.org/dox/TopicWritingEfficientProductExpression.html)

I had attempted using matrices with fixed sizes, such as `Matrix4f`, instead of `MatrixXd`, because it is supposed to be more efficient with smaller dimensions. According to the [Eigne documentation](https://eigen.tuxfamily.org/dox/group__tutorialmatrixclass.html), the fixed sizes are created in the stack, so it can save time creating objects in the heap. However, I have abandoned that approach because mixing fixed sizes and dynamic sizes produced errors event if the dimensions were correct. Many examples in the starter code uses the dynamic sizes, so I kept creating the dynamic sizes unconsciously. This may be also problematic if code is shared by multiple users and others did the same thing as I did.

In order to reduce the number of object creation and copy assignments, I pass `KFState` by reference as shown above in the functions `ProcessMeasurement()`, `Predict()`, and `Update()`. If I had chosen to pass them by value, the functions would have looked like this:

```c++
KFState KF::ProcessMeasurement(const KFState& stateIn, float dt, const Eigen::VectorXd& z) {
  KFState statePred = Predict(stateIn, dt);
  KFState stateOut = Update(statePred, dt, z);
  return stateOut;
}

KFState KF::Predict(const KFState& stateIn, float dt) {
  // Prediction calculation is shared by all subclasses.
  set_F(dt);
  set_Q(dt);

  KFState stateOut;
  stateOut.x = F_ * stateIn.x;
  stateOut.P = F_ * stateIn.P * F_.transpose() + Q_;

  return stateOut;
}
```

There are many intermediate objects created, `statePred` and `stateOut`, and they are all created in the heaps because `x` and `P` are of dynamic sizes, leadding to inefficiency. And these functions are called whenever there is a new measurement, so the inefficiency will add up quickly. Despite the inefficiency, there seems to be an advantage in readability because it shows what each function returns, therefore what it does. In my actual implementation with passing by reference, one of the input parameters, `state`, is being modified within functions, so it is less obvious what each function does for the users of the functions. However, I thought the advantage not creating objects on the heap would outweigh the small convenience of the alternative.

### Flexibility

The class FusionLaserRadar comes with two private variables, `use_laser_` and `use_radar_`. By changing theit values, one can easily turn on or off either sensors and not use them for state estimation. For each measurement, a function `SensorIsOff()` is called to check whether to use the sensor. From the command line, one can decide which sensor to use as following:

```bash
$ ./ExtendedKF        (use both sensors)
$ ./ExtendedKF L      (use lidar only)
$ ./ExtendedKF R      (use radar only)
```

### Scalability

One aspect of scalable code is easiness of adding new functionality. As long as the state of the system is described by the four dimensional state vector `x (px, py, vx, vy)` and its covariant matrix, a new class of kalman filter can be added as a subclass of `KF`.
The new class can reuse the `Predict()` function already written in `KF`. The calls to `Predict()` and `Update()` are already defined in `KF::ProcessMeasurement()`.
All that needs to be done is to write the function `Update()` for the new class, add an instance in `FusionLaserRadar`, and write a call to the interface `KF::ProcessMeasurement()`.

