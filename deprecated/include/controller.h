// Compute control input given desired trajectory
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>

namespace controller
{
  class Controller
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // state vector
    Eigen::VectorXd cur_state_;

    // constructor
    Controller();

    // Initialize
    void Init();

    // compute control action given desired state
    void ComputeAction(const Eigen::VectorXd& desired_state);

    void MockComputeAction(const Eigen::VectorXd& desired_state);
  };

} // namespace controller


#endif