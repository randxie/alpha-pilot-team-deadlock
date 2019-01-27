// Compute control input given desired trajectory
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>

namespace controller
{
  constexpr int STATE_DIM = 12;

  class Controller
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Counter for keeping track of trajectory position
    Eigen::VectorXd cur_state_;

    // constructor
    Controller();

    // Initialize
    void Init();
    
    // Add RGBD camera settings to scene.
    void ComputeAction(const Eigen::VectorXd& desired_state);

    void MockComputeAction(const Eigen::VectorXd& desired_state);
  };

} // namespace controller


#endif