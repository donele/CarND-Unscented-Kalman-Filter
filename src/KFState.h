#ifndef KFState_H_
#define KFState_H_
#include "Eigen/Dense"

class KFState {
public:
  Eigen::VectorXd x;
  Eigen::MatrixXd P;

  /**
  * Constructor
  */
  KFState();

  /**
  * Destructor
  */
  virtual ~KFState();
};

#endif
