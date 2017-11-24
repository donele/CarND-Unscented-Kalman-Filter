# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

An [Unscented Kalman Filter](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf) is impelmented to estimate the state of a moving object.
A constant turn rate and velocity magnitude model (CTRV) is used to represent the state.
Measurements from a radar and a lidar are combined.

A CTRV state is given by

\[x = [p_x, p_y, v, \psi, \dot{\psi}]^T,\]

and $$P$$ is its covariance matrix. The dynamic system of the state is given by

$$x_{k+1} = F(x_k, v_k)$$
$$y_k = H(x_k, n_k)$$,

where $$x_k$$ represent the unobserved state of the system and $$y_k$$ is the observed signal. The process noise $$v_k$$ and measurement noise $$n_k$$ are not nessesarily additive. The time transition of the system is obtained by an integral

$$x_{k+1} = x_k + \int^{t_{k+1}}_{t_k} \dot{x(t)}dt.$$

The noise $$v_k$$ follows a covariance matrix

$$Q = \begin{bmatrix} \nu_a & 0 \\ 0 & \ddot{\psi} \end{bmatrix}$$

The measurement $$H$$ for the lidar is linear operator,

$$H = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 \end{bmatrix}.$$

The measurement vector for the radar $$z_{k+1|k} = [\rho, \phi, \dot{\rho}]^T$$ follows $$z_{k+1|k} = h(x_{k+1}) + w_{k+1}$$, and $$h$$ satisfies

$$\rho = \sqrt{p_x^2 + p_y^2}$$

$$\phi = \arctan(p_y / p_x)$$

$$\dot{\rho} = \frac{p_x \cos(\phi) v + p_y \sin(\phi) v}{\sqrt{p_x^2 + p_y^2}}$$

To make a state prediction with an unscented kalman filter, a sigma matrix is constructed.

$$X_{k|k} = [x_{k|k} x_{k|k} + \sqrt{(\lambda + n_x)P_{k|k} x_{k|k}} - \sqrt{(\lambda + n_x)P_{k|k}}].$$

The sigma matrix is then propagated through $$F$$:

$$X_{k|k-1} = F(X_{k-1|k-1}).$$

The mean and the covariance of calculated as following.

$$x_{k+1|k} = \sum_{i=1}w_i X_{k+1|i,i}$$

$$P_{k+1|k} = \sum_{i=1}w_i (X_{k+1|k,i} - x_{k+1|k}) (X_{k+1|k,i} - x_{k+1|k})^T,$$

where

$$
w_i =
\begin{cases}
 & \frac{\lambda}{\lambda + n_a}, i = 1 \\
 & \frac{1}{2(\lambda + n_a)}, i = 2, ...
\end{cases}
$$

The predicted sigma points $$X_{k|k-1}$$ is then transformed into the measurement space to form $$Z_{k+1|k}$$, and the predicted measurement meand and the covariances are calculated as following.

$$z_{k+1|k} = \sum_{i=1} w_i Z_{k+1|k,i}$$

$$S_{k+1|k} = \sum_{i=1} w_i (Z_{k+1|k,i} - z_{k+1|k}) (Z_{k+1|k,i} - z_{k+1|k})^T + R,$$

where R for the radar is

$$R = \begin{bmatrix} \sigma_{\rho}^2 & 0 & 0 \\ 0 & \sigma_{\phi}^2 & 0 \\ 0 & 0 & \sigma_{\dot{\rho}}^2 \end{bmatrix},$$

and for the lidar,

$$R = \begin{bmatrix} \sigma_{p_x}^2 & 0 \\ 0 & \sigma_{p_y}^2 \end{bmatrix}.$$

The update for the measurement is finally done as following.

$$x_{k+1|k+1} = x_{k+1|k} + K_{k+1|k}(z_{k+1} - z_{k+1|k})$$

$$P_{k+1|k+1} = P_{k+1|k} - K_{k+1|k} S_{k+1|k} K_{k+1|k}^T,$$

where

$$T_{k+1|k} = \sum_{i=1}^{n_{\sigma}} w_i (X_{k+1|k,i} - x_{k+1|k}) (Z_{k+1|k,i} - z_{k+1|k})^T$$

and the kalman gain

$$K_{k+1|k} = T_{k+1|k} S_{k+1|k}^{-1}.$$


## Tuning Process Noise
## Result


The RMSE of the kalman filters for the first data set is as following.

|           |px       |py      |vx      |vy      |
|:---------:|:-------:|:------:|:------:|:------:|
|Laser+Radar|0.0600   |0.0937  |0.3847  |0.2600  |
|Laser only |0.1648   |0.1057  |0.5223  |0.3285  |
|Radar only |0.2111   |0.2597  |0.4648  |0.3088  |



According to a [study](https://www.pdx.edu/ibpi/sites/www.pdx.edu.ibpi/files/Bicycle%20Performance%20Forthcomming%202013.pdf), the average speed and acceleration of a bicycle in a city are 5 m/s and 0.5 m/s^2.  




























