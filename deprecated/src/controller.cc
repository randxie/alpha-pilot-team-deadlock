#include "controller.h"

namespace controller {
  Controller::Controller(): cur_state_(12) {
  };

  void Controller::Init() {
    // State representation: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
    cur_state_ << 0,0,0,0,0,0,0,0,0,0,0,0;
  }
}