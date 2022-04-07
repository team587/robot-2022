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

class VisionContainer
{
  private:

  volatile double yaw;
  volatile double pitch;
  volatile bool hasTarget;
  const static int MAXDISTANCES = 15;
  const static int MINANGLE = -7;
  const static int MINSPEED = 0;
  constexpr static double angleConversion = .546;
  VisionDistance visionDistances[MAXDISTANCES];
  int lastDistance = -1;

  public:

  VisionContainer() {
    int count = 0;
    visionDistances[count++] = VisionDistance(0, .85, 25, .6);
    visionDistances[count++] = VisionDistance(.8, 1.16, 20, .6);
    visionDistances[count++] = VisionDistance(1.11, 1.63, 15, .6);
    visionDistances[count++] = VisionDistance(1.57, 1.95, 11, .64);
    visionDistances[count++] = VisionDistance(1.90, 2.18, 10, .65);
    visionDistances[count++] = VisionDistance(2.13, 2.38, 7, .65);
    visionDistances[count++] = VisionDistance(2.33, 2.54, 6, .66); //could be a problem child
    visionDistances[count++] = VisionDistance(2.48, 2.50, 5, .67);
    visionDistances[count++] = VisionDistance(2.49, 2.78, 5, .68);
    visionDistances[count++] = VisionDistance(2.73, 3.11, -3, .72);
    visionDistances[count++] = VisionDistance(3.06, 3.26, -5, .76);
    visionDistances[count++] = VisionDistance(3.21, 3.47, -6, .82); //could be a problem child
    visionDistances[count++] = VisionDistance(3.42, 3.94, -7, .88);
    visionDistances[count++] = VisionDistance(3.89, 10, -7, .90);
    visionDistances[count++] = VisionDistance(1, 0, -1, -1.0); // For if there is no target
  };

  void start() {
    frc::SmartDashboard::PutString("Debug", "Vision Thread Pre-Start");
    std::thread m_thread(&VisionContainer::VisionThread, this);
    m_thread.detach();
  };

  double getHasTarget() { return hasTarget; }
  double getYaw() { return yaw; }
  double getPitch() { return pitch; }

  double getTurretAngle(double currentAngle) {
    double newTurretAngle = currentAngle;
    if (yaw > .5 || yaw < -.5) {
      newTurretAngle = yaw + currentAngle;
      /*if (newTurretAngle > 180.0) {
        newTurretAngle = 180.0;
      } else if (newTurretAngle < 45.0) {
        newTurretAngle = 45.0;
      }*/
    }
    newTurretAngle = fmax(newTurretAngle, 45);
    newTurretAngle = fmin(newTurretAngle, 180);
    return newTurretAngle;
  }

  double getDistance(double currentAngle) {
    return photonlib::PhotonUtils::CalculateDistanceToTarget(
        Camerapos::cam_height_meters, Camerapos::goal_height_meters, units::degree_t((currentAngle * angleConversion) + Camerapos::angle_offset),
        units::degree_t(pitch)).value();
  }

  VisionDistance* getVisionDistance(double distance) {
    if(lastDistance != -1) {
      if(visionDistances[lastDistance].isDistance(distance)) {
        return &visionDistances[lastDistance];
      }
    }
    for(int x = 0; x < MAXDISTANCES; x++) {
      if(visionDistances[x].isDistance(distance)) {
        lastDistance = x;
        return &visionDistances[x];
      }
    }
    return &visionDistances[MAXDISTANCES-1];
  }

  double getHoodAngle(double currentAngle) {
    double distance = getDistance(currentAngle);
    VisionDistance* visionDistance = getVisionDistance(distance);
    if(visionDistance->m_hoodAngle >= MINANGLE) {
      return visionDistance->m_hoodAngle;
    } else {
      return MINANGLE;
    }
  }

  double getShooterSpeed(double currentAngle) {
    double distance = getDistance(currentAngle);
    VisionDistance* visionDistance = getVisionDistance(distance);
    if(visionDistance->m_shooterSpeed > MINSPEED) {
      return visionDistance->m_shooterSpeed;
    } else {
      return MINSPEED;
    }
  }

private:

  void VisionThread()
  {
    photonlib::PhotonCamera m_camera{"mmal_service_16.1"};
    //frc::SmartDashboard::PutString("Debug", "Vision Thread Start");
    while (true) {
      //frc::SmartDashboard::PutString("Debug", "Vision Thread running");
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
        //std::cout <<"Target Pitch: " << pitch << "Yaw: " << yaw << "\n";
        
      } else {
        //std::cout << "Has no target";
      }
      usleep(100000);
    }
  };
};
