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
#include "wpi/span.h"

class VisionContainer
{
  private:

  volatile double yaw;
  volatile double pitch;
  volatile bool hasTarget;
  const static int MAXDISTANCES = 16;
  const static int MINANGLE = 4;
  const static int MINSPEED = 0;
  constexpr static double angleConversion = .61;
  VisionDistance visionDistances[MAXDISTANCES];
  int lastDistance = -1;
  double manualoffset = 2.26;

  public:

  VisionContainer() {
    int count = 0;
    double offset = 11;
    visionDistances[count++] = VisionDistance(0, .85, 20+offset, .6);
    visionDistances[count++] = VisionDistance(.8, 1.16, 17+offset, .65);
    visionDistances[count++] = VisionDistance(1.11, 1.63, 2+offset, .65);
    visionDistances[count++] = VisionDistance(1.57, 1.95, 0+offset, .67);
    visionDistances[count++] = VisionDistance(1.90, 2.2, -3+offset, .7);
    visionDistances[count++] = VisionDistance(2.15, 2.38, -3+offset, .75);
    visionDistances[count++] = VisionDistance(2.33, 2.54, -5+offset, .8); //could be a problem child
    visionDistances[count++] = VisionDistance(2.48, 2.50, -5+offset, .8);
    visionDistances[count++] = VisionDistance(2.49, 2.78, -6+offset, .84);
    visionDistances[count++] = VisionDistance(2.73, 3.11, -8+offset, .86);
    visionDistances[count++] = VisionDistance(3.06, 3.26, -8+offset, .87);
    visionDistances[count++] = VisionDistance(3.21, 3.47, -8+offset, .88); //could be a problem child
    visionDistances[count++] = VisionDistance(3.42, 3.94, -8+offset, .89);
    visionDistances[count++] = VisionDistance(3.89, 4.2, -8+offset, .98);
    visionDistances[count++] = VisionDistance(4.3, 10, -8+offset, .95);
    visionDistances[count++] = VisionDistance(1, 0, 15, -1.0); // For if there is no target
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
      frc::SmartDashboard::PutNumber("thought angle", (currentAngle-manualoffset) * angleConversion + Camerapos::angle_offset);
    return photonlib::PhotonUtils::CalculateDistanceToTarget(
        Camerapos::cam_height_meters, Camerapos::goal_height_meters, units::degree_t(((currentAngle-manualoffset) * angleConversion) + Camerapos::angle_offset),
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
    return lastDistance != -1 ? &visionDistances[lastDistance] : &visionDistances[0];//&visionDistances[MAXDISTANCES-1];
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
      
      if (result.HasTargets())
      {
        
        // Does other calculations
        //photonlib::PhotonTrackedTarget target = result.GetBestTarget();

        wpi::span<const photonlib::PhotonTrackedTarget> targets = result.GetTargets();
        hasTarget = (targets.size()>1);
        if(hasTarget){
          double yawtobe = 0.0;
          double pitchTobe = 0.0;
          if(fabs(targets[0].GetPitch()-targets[1].GetPitch())>4.0){
            for(int county = 1; county<targets.size(); county++){
              yawtobe+=targets[county].GetYaw();
              pitchTobe+=targets[county].GetPitch();
            }
            yaw = yawtobe/(targets.size()-1.0);
            pitch = targets[1].GetPitch();
            //pitch = pitchTobe/(targets.size()-1.0);
          }
          else{
            for(int county = 0; county<targets.size(); county++){
              yawtobe+=targets[county].GetYaw();
              pitchTobe+=targets[county].GetPitch();
            }
            yaw = yawtobe/targets.size();
            pitch = targets[0].GetPitch();
            //pitch = pitchTobe/targets.size();
          }
          
        } 
        
        //std::cout <<"Target Pitch: " << pitch << "Yaw: " << yaw << "\n";
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
        //std::cout <<"Target" << pitch << yaw;
        
      } else {
        //std::cout << "Has no target";
      }
      usleep(20000);
    }
  };
};
