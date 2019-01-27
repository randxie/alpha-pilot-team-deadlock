#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "simulator_client.h"

namespace simulator
{

#define SHOW_DEBUG_IMAGE_FEED true

// Example consumers and publishers
inline void ImageConsumer(SimulatorClient *self)
{
  while (true)
  {
    // Wait for render result (blocking).
    unity_incoming::RenderOutput_t renderOutput = self->flight_goggles_.handleImageResponse();

    // Display result
    if (SHOW_DEBUG_IMAGE_FEED)
    {
      cv::imshow("Debug RGB", renderOutput.images[0]);
      cv::imshow("Debug D", renderOutput.images[1]);
      cv::waitKey(1);
    }
  }
}

inline void PosePublisher(SimulatorClient *self)
{
  // Sends render requests to FlightGoggles indefinitely
  while (true)
  {
    // Update camera position
    self->UpdateCameraTrajectory();
    // Update timestamp of state message (needed to force FlightGoggles to rerender scene)
    self->flight_goggles_.state.utime = self->flight_goggles_.getTimestamp();
    // request render
    self->flight_goggles_.requestRender();
    // Throttle requests to framerate.
    usleep(1e6 / self->flight_goggles_.state.maxFramerate);
  }
}

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