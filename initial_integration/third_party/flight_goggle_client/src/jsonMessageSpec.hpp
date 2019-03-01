#ifndef UNITYMESSAGESPEC_H
#define UNITYMESSAGESPEC_H
/**
 * @file   jsonMessageSpec.hpp
 * @author Winter Guerra
 * @brief  Defines the json structure of messages going to and from Unity.
 */

// Message/state struct definition

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "json.hpp"
using json = nlohmann::json;

namespace unity_outgoing
{

struct Camera_t
{
  std::string ID;
  // Position and rotation use Unity left-handed coordinates.
  // Z North, X East, Y up.
  // E.G. East, Up, North.
  std::vector<double> position;
  std::vector<double> rotation;
  // Metadata
  int channels;
  bool isDepth;
  int outputIndex;
};

// Window class for decoding the ZMQ messages.
struct Object_t
{
  std::string ID;
  std::string prefabID;
  std::vector<double> position;
  std::vector<double> rotation;
  // Metadata
  std::vector<double> size;
};

struct StateMessage_t
{
  // Scene/Render settings
  int maxFramerate = 60; 
  bool sceneIsInternal = true;
  // Scene choices in v1.4.1
  // std::string sceneFilename = "Hazelwood_Loft_Full_Night";
  // std::string sceneFilename = "Hazelwood_Loft_Full_Day";
  // std::string sceneFilename = "Butterfly_World";
  // std::string sceneFilename = "NYC_Subway";
  // std::string sceneFilename = "Museum_Day";
  std::string sceneFilename = "Museum_Day_Small";

  bool compressImage = false; // Deprecated. Will be removed in the future.
  
  // Frame Metadata
  int64_t utime;
  int camWidth = 1024;
  int camHeight = 768;
  float camFOV = 70.0f;
  double camDepthScale = 0.20; // 0.xx corresponds to xx cm resolution
  
  // CTAA AntiAliasing Settings
  float temporalJitterScale = 0.1f; // [0.0, 0.5] default 0.475  
  int temporalStability = 8; // int [3,16] default 8            
  float hdrResponse = 0.001f; // [0.001, 1.0] default 0.001        
  float sharpness = 9.5f; // [0.0, 10.0] default 9.5               
  float adaptiveEnhance = 0.32f; // [0.2, 0.5] default 0.32        
  float microShimmerReduction = 3.0f; // [0.01, 10.0] default 3.0  
  float staticStabilityPower = 0.5f; // [0.0, 1.0] default 0.5     
  
  // Object state update
  std::vector<Camera_t> cameras;
  std::vector<Object_t> objects;
};

// Json constructors

// StateMessage_t
inline void to_json(json &j, const StateMessage_t &o)
{
  j = json{// Initializers
           {"maxFramerate", o.maxFramerate},
           {"sceneIsInternal", o.sceneIsInternal},
           {"sceneFilename", o.sceneFilename},
           {"compressImage", o.compressImage},
           // CTAA settings
           {"temporalJitterScale", o.temporalJitterScale},
           {"temporalStability", o.temporalStability},
           {"hdrResponse", o.hdrResponse},
           {"sharpness", o.sharpness},
           {"adaptiveEnhance", o.adaptiveEnhance},
           {"microShimmerReduction", o.microShimmerReduction},
           {"staticStabilityPower", o.staticStabilityPower},
           // Frame Metadata
           {"utime", o.utime},
           {"camWidth", o.camWidth},
           {"camHeight", o.camHeight},
           {"camFOV", o.camFOV},
           {"camDepthScale", o.camDepthScale},
           // Object state update
           {"cameras", o.cameras},
           {"objects", o.objects}};
}

// Camera_t
inline void to_json(json &j, const Camera_t &o)
{
  j = json{{"ID", o.ID},
           {"position", o.position},
           {"rotation", o.rotation},
           {"channels", o.channels},
           {"isDepth", o.isDepth},
           {"outputIndex", o.outputIndex}};
}

// Object_t
inline void to_json(json &j, const Object_t &o)
  {
    j = json{
      {"ID", o.ID},
      {"prefabID", o.prefabID},
      {"position", o.position},
      {"rotation", o.rotation},
      {"size", o.size}
    };
  }
}

// Struct for returning metadata from Unity.
namespace unity_incoming
{

struct RenderMetadata_t
{
  // Metadata
  int64_t utime;
  int camWidth;
  int camHeight;
  bool isCompressed;
  double camDepthScale;
  // Object state update
  std::vector<std::string> cameraIDs;
  std::vector<int> channels;
};

// Json Parsers

// RenderMetadata_t
inline void from_json(const json &j, RenderMetadata_t &o)
{
  o.utime = j.at("utime").get<int64_t>();
  o.camWidth = j.at("camWidth").get<int>();
  o.camHeight = j.at("camHeight").get<int>();
  o.camDepthScale = j.at("camDepthScale").get<double>();
  o.isCompressed = j.at("isCompressed").get<bool>();
  o.cameraIDs = j.at("cameraIDs").get<std::vector<std::string>>();
  o.channels = j.at("channels").get<std::vector<int>>();
}

// Struct for outputting parsed received messages to handler functions
struct RenderOutput_t
{
  RenderMetadata_t renderMetadata;
  std::vector<cv::Mat> images;
};
}

#endif