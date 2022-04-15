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
#include <frc/DriverStation.h>


class BallVisionContainer
{
  private:


  //Ball variables
  volatile double ballYaw;
  volatile double ballPitch;
  volatile bool ballHasTarget;

  public:

  BallVisionContainer() {};

  void start() {
    frc::SmartDashboard::PutString("Debug", "Ball Vision Thread Pre-Start");
    std::thread m_thread(&BallVisionContainer::VisionThread, this);
    m_thread.detach();
  };

  double getBallYaw() { return ballYaw; }
  double getBallPitch() { return ballPitch; }


photonlib::PhotonCamera*m_ballCamera;

  void SetColorPipeline(){
  frc::DriverStation::Alliance alliance = frc::DriverStation::GetAlliance();
  if (frc::DriverStation::Alliance::kBlue == alliance) {
    m_ballCamera->SetPipelineIndex(1);
  } else {
    m_ballCamera->SetPipelineIndex(0);
  }
  
}

private:

  void VisionThread()
  {
    m_ballCamera = new photonlib::PhotonCamera{"ball"};
    SetColorPipeline();
    //frc::SmartDashboard::PutString("Debug", "Vision Thread Start");
    while (true) {
      //frc::SmartDashboard::PutString("Debug", "Vision Thread running");
      // wpi::outs() << "Lock Exec\n";
      photonlib::PhotonPipelineResult ballResult = m_ballCamera->GetLatestResult();
      ballHasTarget = ballResult.HasTargets();
      if (ballHasTarget)
      {
        photonlib::PhotonTrackedTarget ballTarget = ballResult.GetBestTarget();
        ballYaw = ballTarget.GetYaw();
        ballPitch = ballTarget.GetPitch();
      }
      usleep(10000);
    }
  };
};
