#ifndef FusionLaserRadar_H_
#define FusionLaserRadar_H_

#include "measurement_package.h"
#include "KF.h"
#include "KFState.h"
#include "tools.h"

class FusionLaserRadar {
public:
  /**
  * Constructor.
  */
  FusionLaserRadar();

  /**
  * Destructor.
  */
  virtual ~FusionLaserRadar();

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
  * Kalman Filter update and prediction math lives in here.
  */
  KF* kfLaser_;
  KF* kfRadar_;
  KFState state_;

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

#endif /* FusionLaserRadar_H_ */
