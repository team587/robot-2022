// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSubsystem.h"
#include <frc/controller/HolonomicDriveController.h>
#include <photonlib/PhotonUtils.h>
#include <photonlib/PhotonCamera.h>
#include <wpi/span.h>
#include <units/length.h>
#include <math.h>
#include <thread>
#include <photonlib/PhotonUtils.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "robot.h"

#ifndef EXCLUDE_PATHPLANNER

#endif

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class VisionContainer
{
  class VisionDistance {
    public:
    VisionDistance();
    VisionDistance(double minDistance, double maxDistance, double hoodAngle,  double shooterSpeed){
      m_minDistance = minDistance;
      m_maxDistance = maxDistance;
      m_hoodAngle = hoodAngle;
      m_shooterSpeed = shooterSpeed;
    };
    double m_minDistance;
    double m_maxDistance;
    double m_hoodAngle;
    double m_shooterSpeed;
    bool isDistance(double distance){
      if(distance >= m_minDistance && distance < m_maxDistance){
         return true;
      }
      return false;
    }
  };

public:
  VisionContainer() {
    int count = 0;

    visionDistances[count++] = VisionDistance(0, 1, 0, .4);
    visionDistances[count++] = VisionDistance(1, 2, 0, .4);
    visionDistances[count++] = VisionDistance(2, 3, 0, .4);
    visionDistances[count++] = VisionDistance(3, 4, 0, .4);
    visionDistances[count++] = VisionDistance(4, 5, 0, .4);
    visionDistances[count++] = VisionDistance(5, 6, 0, .4);
    visionDistances[count++] = VisionDistance(6, 7, 0, .4);
    visionDistances[count++] = VisionDistance(7, 8, 0, .4);
    visionDistances[count++] = VisionDistance(8, 9, 0, .4);
    visionDistances[count++] = VisionDistance(9, 10, 0, .4);

  };

  void start()
  {
    std::thread m_thread(VisionThread);
    m_thread.detach();
  };

  double getTargetAngle(double currentAngle)
  {
    double newangle = currentAngle;
    if (yaw > 1.0 || yaw < -1.0)
    {
      newangle = yaw + currentAngle;

      if (newangle > 180.0)
      {
        newangle = 180.0;
      }
      else if (newangle < 0.0)
      {
        newangle = 0.0;
      }
    }
    return newangle;
  }
  double getDistance(double currentAngle){
    return photonlib::PhotonUtils::CalculateDistanceToTarget(
        Camerapos::cam_height_meters, Camerapos::goal_height_meters, units::degree_t(currentAngle),
        units::degree_t(pitch)).value();
  }
  VisionDistance* getVisionDistance(double distance){
    for(int x = 0; x<sizeof(visionDistances)/sizeof(visionDistances[0]); x++){
      if(visionDistances[x].isDistance(distance)){
        return &visionDistances[x];
      }
    }
    return NULL;
  }
  double getHoodAngle(double currentAngle)
  {
    double newangle = currentAngle;
    double distance = getDistance(currentAngle);
    VisionDistance* visionDistance = getVisionDistance(distance);
    return visionDistance->m_hoodAngle;
  }
  double getShooterSpeed(double currentAngle){
    double distance = getDistance(currentAngle);
    VisionDistance* visionDistance = getVisionDistance(distance);
    return visionDistance->m_shooterSpeed;
  }

private:
  volatile static double yaw;
  volatile static double pitch;
  static const int MAXDISTANCES = 10;
  VisionContainer::VisionDistance visionDistances[MAXDISTANCES];

  static void VisionThread()
  {
    photonlib::PhotonCamera m_camera{"mmal_service_16.1"};
    while (true)
    {

      // wpi::outs() << "Lock Exec\n";
      photonlib::PhotonPipelineResult result = m_camera.GetLatestResult();
      // wpi::outs() << "Camera is connected\n";
      // frc::SmartDashboard::PutBoolean("has a target", result.HasTargets());
      if (result.HasTargets())
      {
        // Does other calculations
        photonlib::PhotonTrackedTarget target = result.GetBestTarget();
        yaw = target.GetYaw();
        pitch = target.GetPitch();
        // wpi::outs() << std::to_string(photonlib::PhotonUtils::CalculateDistanceToTarget(
        // Camerapos::cam_height_meters, Camerapos::goal_height_meters, Camerapos::pitch,
        // units::degree_t{result.GetBestTarget().GetPitch()}).value()) << "\n";
        // frc::SmartDashboard::PutNumber("Yaw", target.GetYaw());
        // frc::SmartDashboard::PutNumber("Pitch", target.GetPitch());
        // frc::SmartDashboard::PutNumber("Skew", target.GetSkew());
        // frc::SmartDashboard::PutNumber("Distance using photon", photonlib::PhotonUtils::CalculateDistanceToTarget(
        // Camerapos::cam_height_meters, Camerapos::goal_height_meters, Camerapos::pitch,
        // units::degree_t{result.GetBestTarget().GetPitch()}).value());
        // command to find distance when we know the hood angle:
        // frc::SmartDashboard::PutNumber("Distance using photon", photonlib::PhotonUtils::CalculateDistanceToTarget(
        // Camerapos::cam_height_meters, Camerapos::goal_height_meters, units::degree_t(m_shooter->getHoodAngle()),
        // units::degree_t(result.GetBestTarget().GetPitch())).value());
        
      }
    }
  };
};
