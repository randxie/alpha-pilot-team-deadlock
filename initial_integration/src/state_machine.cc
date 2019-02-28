#include <cmath>
#include "state_machine.h"

namespace state_machine
{
void StateMachine::RefreshData(cv::Mat& images, Eigen::VectorXd& imu_data)
{
  // send data to slam module to get latest location and Euler angle

  // update controller internal state
  for(int i=0; i < 6; i++) {
    controller_.cur_state_[i] = imu_data[i];
  }
}

float StateMachine::GetHeight()
{
  return controller_.cur_state_[2];
}

void StateMachine::TryStateChange(SystemState desired_state)
{
  switch (state_)
  {
    case SystemState::start:
      if (desired_state == SystemState::exploring)
      {
        if (std::abs(GetHeight() - DEFAULT_HOVER_HEIGHT) < DEFAULT_HOVER_DIST_ERROR)
        {
          state_ = SystemState::exploring;
        }
      }
      break;
  }
}
} // namespace state_machine