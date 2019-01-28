#include <iostream>
#include <unistd.h>
#include "controller.h"
#include "state_machine.h"
#include "simulator_client.h"
#define SHOW_DEBUG_IMAGE_FEED true

// Example consumers and publishers
void ImageConsumer(simulator::SimulatorClient *self, controller::Controller *controller_client)
{
  while (true)
  {
    // Wait for render result (blocking).
    unity_incoming::RenderOutput_t renderOutput = self->flight_goggles_.handleImageResponse();

    // Display result
    if (SHOW_DEBUG_IMAGE_FEED)
    {
      //cv::imshow("Debug RGB", renderOutput.images[0]);
      cv::imshow("Debug D", renderOutput.images[1]);
      cv::waitKey(1);
    }
  }
}

void KeyboardPublisher(controller::Controller *controller_client)
{
  // system("stty raw"); 
  while (true)
  {
    char c = getchar();
    switch(c) {
      case 'w': controller_client->cur_state_(2) += 0.1;
                break;
      case 's': controller_client->cur_state_(2) -= 0.1;
                break;
      case 'a': controller_client->cur_state_(0) += 0.1;
                break;
      case 'd': controller_client->cur_state_(0) -= 0.1;
                break;
      case 'f': controller_client->cur_state_(1) -= 0.1;
                break;
      case 'b': controller_client->cur_state_(1) += 0.1;
                break;
      case 'l': controller_client->cur_state_(3) += 0.1;
                break;
      case 'r': controller_client->cur_state_(3) -= 0.1;
                break;
    }
  }
  // system("stty cooked"); 
}

void PosePublisher(simulator::SimulatorClient *self, controller::Controller *controller_client)
{
  // Sends render requests to FlightGoggles indefinitely
  Eigen::VectorXd desired_state(12);
  desired_state << 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  while (true)
  {
    // Update camera position
    self->SetCameraState(controller_client->cur_state_);

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
  simulator_client.flight_goggles_.state.sceneFilename = "Hazelwood_Loft_Full_Day";

  // Fork sample render request thread
  // will request a simple circular trajectory
  std::thread pose_publisher_thread(PosePublisher, &simulator_client, &controller_client);

  // Fork a sample image consumer thread
  std::thread image_consumer_thread(ImageConsumer, &simulator_client, &controller_client);

  // keyboard event
  std::thread keyboard_thread(KeyboardPublisher, &controller_client);

  // Spin
  while (true)
  {
    sleep(1);
  }

  return 0;
}