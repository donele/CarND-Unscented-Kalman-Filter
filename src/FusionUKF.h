#ifndef FusionUKF_H_
#define FusionUKF_H_

#include "measurement_package.h"
#include "UKF.h"
#include "StateCTRV.h"
#include "tools.h"

class FusionUKF {
public:
  /**
  * Constructor.
  */
  FusionUKF(float std_a, float std_yawdd, float p1, float p2, float p3, float p4, float p5);

  /**
  * Destructor.
  */
  virtual ~FusionUKF();

  /**
  * Decide whether to use the laser sensor.
  */
  void UseLaser(bool use=true) {use_laser_ = use;}

  /**
  * Decide whether to use the radar sensor.
  */
  void UseRadar(bool use=true) {use_radar_ = use;}

  /**
  * Check if the sensor is allowed to be used.
  */
  bool SensorIsOff(const MeasurementPackage &measurement_pack);

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Get state in Constant Veocity format
   */
  Eigen::VectorXd GetStateCV();

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  UKF* kfLaser_;
  UKF* kfRadar_;
  StateCTRV state_;

private:
  // Decide which sensor to use (Use both by default)
  bool use_laser_;
  bool use_radar_;

  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;

  /**
  * Initialize the state from the first measurement
  */
  void Init(const MeasurementPackage &measurement_pack);

};

#endif /* FusionUKF_H_ */
