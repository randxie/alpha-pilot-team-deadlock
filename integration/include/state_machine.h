#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "controller.h"
#include "localizer.h"
#include <opencv2/opencv.hpp>

namespace state_machine
{
constexpr float DEFAULT_HOVER_HEIGHT = 1.5;
constexpr float DEFAULT_HOVER_DIST_ERROR = 0.1;

enum SystemState {
    start,
    exploring,
    planning,
    gate_valiating,
    gate_tracking,
    stop
};

class StateMachine
{
  public:
    // Current system state
    SystemState state_;

    // Controller
    controller::Controller controller_;

    // SLAM
    slam::Localizer localizer_;

    // Set/Get method for state
    void SetState(SystemState state) { state_ = state; };
    const SystemState& GetState() const { return state_; };

    // Action
    void Reset();
    void RefreshData(cv::Mat& images, Eigen::VectorXd& imu_data);
    void TryStateChange(SystemState target_state);

    // Get real states
    float GetHeight();

    // Set initial state as start
    StateMachine() { state_ = SystemState::start; };
};

} // namespace state_machine

#endif