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
#include "VisionDistance.h"
#include <unistd.h>
#include "iostream"
//#include "robot.h"

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
  private:

  volatile double yaw;
  volatile double pitch;
  volatile bool hasTarget;
  const static int MAXDISTANCES = 11;
  constexpr static double angleConversion = 18/30;
  VisionDistance visionDistances[MAXDISTANCES];

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
    visionDistances[count++] = VisionDistance(1, 0, -1, -1.0); // For if there is no target
  };

  void start()
  {
    std::thread m_thread(&VisionContainer::VisionThread, this);
    m_thread.detach();
  };

  double getHasTarget() { return hasTarget; }
  double getYaw() { return yaw; }
  double getPitch() { return pitch; }

  double getTurretAngle(double currentAngle) {
    double newTurretAngle = currentAngle;
    if (yaw > 1.0 || yaw < -1.0) {
      newTurretAngle = yaw + currentAngle;
      if (newTurretAngle > 180.0) {
        newTurretAngle = 180.0;
      } else if (newTurretAngle < 45.0) {
        newTurretAngle = 45.0;
      }
    }
    return newTurretAngle;
  }

  double getDistance(double currentAngle){
    return photonlib::PhotonUtils::CalculateDistanceToTarget(
        Camerapos::cam_height_meters, Camerapos::goal_height_meters, units::degree_t((currentAngle * angleConversion) + Camerapos::angle_offset),
        units::degree_t(pitch)).value();
  }

  VisionDistance* getVisionDistance(double distance){
    for(unsigned int x = 0; x < MAXDISTANCES; x++){
      if(visionDistances[x].isDistance(distance)){
        return &visionDistances[x];
      }
    }
    return &visionDistances[MAXDISTANCES-1];
  }

  double getHoodAngle(double currentAngle)
  {
    //double newangle = currentAngle;
    double distance = getDistance(currentAngle);
    VisionDistance* visionDistance = getVisionDistance(distance);
    
    if(visionDistance->m_hoodAngle>0){
      return visionDistance->m_hoodAngle;
    }
    else{
      return currentAngle;
    }
  }

  double getShooterSpeed(double currentAngle){
    double distance = getDistance(currentAngle);
    VisionDistance* visionDistance = getVisionDistance(distance);
    if(visionDistance->m_shooterSpeed > 0){
      return visionDistance->m_shooterSpeed;
    }
    else{
      return -1;
    }
  }

private:

  void VisionThread()
  {
    photonlib::PhotonCamera m_camera{"mmal_service_16.1"};
    while (true)
    {

      // wpi::outs() << "Lock Exec\n";
      photonlib::PhotonPipelineResult result = m_camera.GetLatestResult();
      // wpi::outs() << "Camera is connected\n";
      // frc::SmartDashboard::PutBoolean("has a target", result.HasTargets());
      hasTarget = result.HasTargets();
      if (hasTarget)
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
        std::cout <<"Target Pitch: " << pitch << "Yaw: " << yaw << "\n";
        
      }
      else{
        std::cout << "Has no target";
      }
      //thread::
      //usleep(15000);
      usleep(.1 * 1000000);
    }
    
  };
};

//double VisionContainer::yaw = 0;
//double VisionContainer::pitch = 0;
