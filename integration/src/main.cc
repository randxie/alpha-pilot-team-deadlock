#include <iostream>
#include <unistd.h>
#include "controller.h"
#include "state_machine.h"
#include "simulator_client.h"
#define SHOW_DEBUG_IMAGE_FEED true

// Example consumers and publishers
void ImageConsumer(simulator::SimulatorClient *self)
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

void PosePublisher(simulator::SimulatorClient *self)
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

int main()
{
  std::cout << "Running test" << std::endl;

  // Add controller into system
  controller::Controller controller_client;
  controller_client.Init();

  // Create client
  simulator::SimulatorClient simulator_client;

  // Instantiate RGBD cameras
  simulator_client.AddCameras();

  // Set scene parameters.
  simulator_client.flight_goggles_.state.maxFramerate = 60;

  /*
  Available scenes: [
    "Hazelwood_Loft_Full_Night"
    "Hazelwood_Loft_Full_Day",
    "Butterfly_World",
    "FPS_Warehouse_Day",
    "FPS_Warehouse_Night",
  ]
   */
  simulator_client.flight_goggles_.state.sceneFilename = "Butterfly_World";

  // Fork sample render request thread
  // will request a simple circular trajectory
  std::thread pose_publisher_thread(PosePublisher, &simulator_client);

  // Fork a sample image consumer thread
  std::thread image_consumer_thread(ImageConsumer, &simulator_client);

  // Spin
  while (true)
  {
    sleep(1);
  }

  return 0;
}