#ifndef FLIGHTGOGGLESTRANSFORMS_H
#define FLIGHTGOGGLESTRANSFORMS_H

/**
 * @brief A set of functions that convert common right handed coordinate 
 * systems used in robotics to the left hand system used by the FlightGoggles 
 * backend (Unity 3D).
 * 
 * @file transforms.hpp
 * @author Winter Guerra <winterg@mit.edu>
 * @date 2018-03-25
 */

////////////////////////////////
// TRANSFORMS
////////////////////////////////

// For transforms
#include <Eigen/Geometry>
#include <Eigen/Dense>

#define PRECISION 10
#define DONTALIGNCOLS 1

// define some eigen types and formats
typedef Eigen::Vector3d Vector3;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Quaternion<double> Quaternionx;
typedef Eigen::Affine3d Transform3;
typedef Eigen::Matrix4d Matrix4;
typedef Eigen::Matrix3d Matrix3;
inline Eigen::IOFormat CSV(PRECISION, DONTALIGNCOLS, ",", ",", "", "", "", "");

/**
 * @brief Converts right hand rule North East Down (NED) global poses to 
 * left handed coordinates (as used by the Unity3D backend of FlightGoggles).
 * 
 * @param NEDworld_T_object 
 * @return Transform3 
 */
inline Transform3 convertNEDGlobalPoseToGlobalUnityCoordinates(
    Transform3 NEDworld_T_object)
{
    // Switch axis to unity axis.
    Matrix4 unity_pose_T_NED_pose;
    // x->z, y->x, z->-y
    // clang-format off
    unity_pose_T_NED_pose <<    0, 1, 0,  0,
                                0, 0, -1, 0,
                                1, 0, 0,  0,
                                0, 0, 0,  1;
    // clang-format on

    Transform3 unity_pose;

    unity_pose.matrix() = unity_pose_T_NED_pose * 
                          NEDworld_T_object.matrix() *
                          unity_pose_T_NED_pose.transpose();

    return unity_pose;
}

/**
 * @brief Converts right hand rule North East Down (NED) poses to left handed 
 * rule coordinates (as used by Unity3D). Takes in an additional parameter 
 * for the translational and rotational offset between the FlightGoggles world
 * and the global NED poses used by the robot.
 * 
 * @deprecated - will be removed soon.
 * 
 * @param NEDworld_T_object 
 * @param unityWorld_T_NEDworld 
 * @return Transform3 
 */
inline Transform3 convertNEDGlobalPoseToGlobalUnityCoordinates(
    Transform3 NEDworld_T_object, Transform3 unityWorld_T_NEDworld)
{
    // Switch axis to unity axis.
    Matrix4 unity_pose_T_NED_pose;
    // x->z, y->x, z->-y
    // clang-format off
    unity_pose_T_NED_pose <<    0, 1, 0,  0,
                                0, 0, -1, 0,
                                1, 0, 0,  0,
                                0, 0, 0,  1;
    // clang-format on

    Transform3 unity_pose;

    unity_pose.matrix() = unity_pose_T_NED_pose * unityWorld_T_NEDworld.matrix() *
                          NEDworld_T_object.matrix() *
                          unity_pose_T_NED_pose.transpose();

    return unity_pose;
}

/**
 * @brief Converts ROS global poses to NED poses. These poses are then converted to 
 * Unity left handed coordinates. Assumes that ROS global coordinates for translation are 
 * East north up (ENU) and robot local coordinates are North west up (NWU). E.g. the robot 
 * local X axis is going out of the camera.  
 * 
 * @param ENUworld_T_object 
 * @return Transform3 
 */
inline Transform3 convertROSToNEDCoordinates(Transform3 ENUworld_T_object)
{
    // Switch ENU axis to NED axis.
    Matrix4 ENU_pose_T_NED_pose;
    // x->y, y->x, z->-z, w->w
    // clang-format off
    ENU_pose_T_NED_pose <<  0, 1, 0,  0,
                            1, 0, 0,  0,
                            0, 0, -1, 0,
                            0, 0, 0,  1;
    // clang-format on

    Transform3 NED_pose;
    NED_pose.matrix() = ENU_pose_T_NED_pose * ENUworld_T_object.matrix() * ENU_pose_T_NED_pose.transpose();

    // Rotate robot pose by -90deg about robot z axis since ROS
    // expects that "X" is forward in robot frame.
    // Quaternionx Y_front_to_X_front_quat(sqrt(0.5f),	0.0f, 0.0f, -sqrt(0.5f));

    // NED_pose = Y_front_to_X_front_quat * NED_pose;


    return NED_pose;
}

// Converts a given LCM drone pose and relative camera pose into a global Unity
// pose for the camera.
/* Move world_T_cam from drone (right handed) coordinate system to Unity (left
handed) coordinate system:
Input vector in unity coordinates goes through the following
transformations:
Unity coordinates -> Drone coordinates -> perform rotation -> unity
coordinates
*/
inline Transform3 convertCameraAndDronePoseToUnityCoordinates(
    Transform3 world_T_body, Transform3 body_T_cam,
    Transform3 unityWorld_T_NEDworld)
{

    Transform3 world_T_cam = world_T_body * body_T_cam;
    Matrix4 cam_T_unitycam;
    // clang-format off
    cam_T_unitycam <<   0, 1, 0, 0,
                        0, 0, 1, 0,
                        1, 0, 0, 0,
                        0, 0, 0, 1;
    // clang-format on

    Transform3 world_T_unitycam;
    world_T_unitycam.matrix() =
        world_T_cam *
        cam_T_unitycam; // Move from z aligned camera to x aligned camera

    // x->z, y->x, z->-y
    Matrix4
        unity_from_drone; /**< Coordinate change from drone coordinate system to
                            unity coordinate system */
    // clang-format off
    unity_from_drone << 0, 1,  0, 0,
                        0, 0, -1, 0,
                        1, 0,  0, 0,
                        0, 0,  0, 1;
    // clang-format on

    Transform3 unityWorld_T_unitycam_unity; /**< Transformation from camera to
                                                world in unity coordinates */
    unityWorld_T_unitycam_unity.matrix() =
        unity_from_drone * unityWorld_T_NEDworld.matrix() *
        world_T_unitycam.matrix() * unity_from_drone.transpose();

    return unityWorld_T_unitycam_unity;
}

#endif