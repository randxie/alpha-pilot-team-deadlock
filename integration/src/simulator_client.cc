#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "simulator_client.h"

namespace simulator
{

// Constructors
SimulatorClient::SimulatorClient()
{
  start_time = flight_goggles_.getTimestamp();
}

void SimulatorClient::AddCameras()
{
  // Prepopulate metadata of cameras (RGBD)
  unity_outgoing::Camera_t cam_rgb;
  cam_rgb.ID = "Camera_RGB";
  cam_rgb.channels = 3;
  cam_rgb.isDepth = false;
  cam_rgb.outputIndex = 0;

  unity_outgoing::Camera_t cam_depth;
  cam_depth.ID = "Camera_D";
  cam_depth.channels = 1;
  cam_depth.isDepth = true;
  cam_depth.outputIndex = 1;

  // Add cameras to persistent state
  flight_goggles_.state.cameras.push_back(cam_rgb);
  flight_goggles_.state.cameras.push_back(cam_depth);
}

// Do a simple circular trajectory
void SimulatorClient::UpdateCameraTrajectory()
{
  double period = 15.0f;
  double r = 2.0f;
  double t = (flight_goggles_.getTimestamp() - start_time) / 1000000.0f;
  double theta = -((t / period) * 2.0f * M_PI);

  Transform3 camera_pose;
  camera_pose.translation() = Vector3(r * cos(theta), r * sin(theta), 1.5f);
  // Set rotation matrix using pitch, roll, yaw
  camera_pose.linear() = Eigen::AngleAxisd(theta - M_PI, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

  // Populate status message with new pose
  flight_goggles_.setCameraPoseUsingROSCoordinates(camera_pose, 0);
  flight_goggles_.setCameraPoseUsingROSCoordinates(camera_pose, 1);
}

} // namespace simulator