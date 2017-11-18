#ifndef StateCTRV_H_
#define StateCTRV_H_
#include "Eigen/Dense"

class StateCTRV {
public:
  Eigen::VectorXd x;
  Eigen::MatrixXd P;

  /**
  * Constructor
  */
  StateCTRV();

  /**
  * Destructor
  */
  virtual ~StateCTRV();
};

#endif
