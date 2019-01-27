#ifndef SIMULATOR_CLIENT_H
#define SIMULATOR_CLIENT_H

#include <FlightGogglesClient.hpp>
// #include <jsonMessageSpec.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <thread>

namespace simulator
{
  class SimulatorClient
  {
  public:
    // FlightGoggles interface object
    FlightGogglesClient flight_goggles_;

    // Counter for keeping track of trajectory position
    int64_t start_time;

    // constructor
    SimulatorClient();

    // Add RGBD camera settings to scene.
    void AddCameras();

    // Do a simple trajectory with the camera
    void UpdateCameraTrajectory();
  };

  inline void ImageConsumer(SimulatorClient *self);
  inline void PosePublisher(SimulatorClient *self);
} // namespace simulator

#endif
