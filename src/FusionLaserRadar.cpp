#include "FusionLaserRadar.h"
#include "tools.h"
#include "Eigen/Dense"
#include "KFLaser.h"
#include "KFRadar.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
		UpdateRadar(measurement_pack);
	else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
		UpdateLaser(measurement_pack);

 	// print the output
 	//cout << "x = " << state_.x << endl;
 	//cout << "P = " << state_.P << endl;
}

bool FusionLaserRadar::SensorIsOff(const MeasurementPackage &measurement_pack) {
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

void FusionLaserRadar::UpdateLaser(const MeasurementPackage &measurement_pack) {
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.;
	state_ = kfLaser_->Update(state_, dt, measurement_pack.raw_measurements_);

	previous_timestamp_ = measurement_pack.timestamp_;
}

void FusionLaserRadar::UpdateRadar(const MeasurementPackage &measurement_pack) {
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.;
	state_ = kfRadar_->Update(state_, dt, measurement_pack.raw_measurements_);

	previous_timestamp_ = measurement_pack.timestamp_;
}
