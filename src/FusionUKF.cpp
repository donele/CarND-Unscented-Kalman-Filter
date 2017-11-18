#include "FusionLaserRadar.h"
#include "KFLaser.h"
#include "KFRadar.h"
#include <iostream>

using namespace std;

/*
 * Constructor.
 */
FusionLaserRadar::FusionLaserRadar()
:use_laser_(true),
use_radar_(true),
is_initialized_(false),
previous_timestamp_(0) {
  kfLaser_ = new KFLaser();
  kfRadar_ = new KFRadar();
}

/**
* Destructor.
*/
FusionLaserRadar::~FusionLaserRadar() {
  delete kfLaser_;
  delete kfRadar_;
}

void FusionLaserRadar::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if(SensorIsOff(measurement_pack))
    return;

  /*****************************************************************************
   *  Initialization
   * Initialize the state x with the first measurement.
   ****************************************************************************/
  if (!is_initialized_) {
    Init(measurement_pack);
    is_initialized_ = true;

    // done initializing, no need to predict or update
    return;
  }

  /*****************************************************************************
   *  Prediction and Update
   * Update the state transition matrix F according to the new elapsed time.
   * Use the sensor type to perform the update step.
   * Update the state and covariance matrices.
   ****************************************************************************/
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    kfRadar_->ProcessMeasurement(state_, dt, measurement_pack.raw_measurements_);
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    kfLaser_->ProcessMeasurement(state_, dt, measurement_pack.raw_measurements_);

  previous_timestamp_ = measurement_pack.timestamp_;

  // print the output
  //cout << "x = " << state_.x << endl;
  //cout << "P = " << state_.P << endl;
}

bool FusionLaserRadar::SensorIsOff(const MeasurementPackage &measurement_pack) {
  // Being able to turn off a sensor may be useful for understanding the effect of each sensors.
  if (!use_radar_ && measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    return true;
  else if (!use_laser_ && measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    return true;
  return false;
}

void FusionLaserRadar::Init(const MeasurementPackage &measurement_pack) {
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    state_.x = tools.Polar2Cartesian(measurement_pack.raw_measurements_);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    state_.x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
  }

  previous_timestamp_ = measurement_pack.timestamp_;
}

