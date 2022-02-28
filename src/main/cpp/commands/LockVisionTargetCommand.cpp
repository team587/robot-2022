// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LockVisionTargetCommand.h"
#include <photonlib/PhotonUtils.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "robot.h"


LockVisionTargetCommand::LockVisionTargetCommand(photonlib::PhotonCamera* camera) {
  wpi::outs() << "Lock Construct\n";
  // Use addRequirements() here to declare subsystem dependencies.
  m_camera = camera;
  //m_shooter = shooter;
  //AddRequirements(m_shooter);
}

// Called when the command is initially scheduled.
void LockVisionTargetCommand::Initialize() {
  wpi::outs() << "Lock Init\n";
}

// Called repeatedly when this Command is scheduled to run
void LockVisionTargetCommand::Execute() {
  wpi::outs() << "Lock Exec\n";
  photonlib::PhotonPipelineResult result = m_camera->GetLatestResult();
  wpi::outs() << "Camera is connected\n";
  frc::SmartDashboard::PutBoolean("has a target", result.HasTargets());
  if(result.HasTargets()){
    //Gets array of targets
    wpi::span<const photonlib::PhotonTrackedTarget> targets = result.GetTargets();
    //finds the ones likely to be of the goal
    double meanPitch = result.GetBestTarget().GetPitch();
    //These may return unit values, if it breaks, that is why
    //This is meant to hold an array of pointers
    const photonlib::PhotonTrackedTarget* goalresults[5] = {};
    int y = 0;
    for(int x = targets.size(); x>0; x--){
      if(abs(targets[x].GetPitch()-meanPitch)>10.0){
        goalresults[y] = &targets[x];
        y++;
      }
    };
    //wpi::outs() << "dist estamate\n"; 
    //wpi::outs() << std::to_string(photonlib::PhotonUtils::CalculateDistanceToTarget(
    //      Camerapos::cam_height_meters, Camerapos::goal_height_meters, Camerapos::pitch,
    //      units::degree_t{result.GetBestTarget().GetPitch()}).value()) << "\n";
    //frc::SmartDashboard::PutNumber("Yaw", target.GetYaw());
    //frc::SmartDashboard::PutNumber("Pitch", target.GetPitch());
    //frc::SmartDashboard::PutNumber("Skew", target.GetSkew());
    //frc::SmartDashboard::PutNumber("Distance", photonlib::PhotonUtils::CalculateDistanceToTarget(
    //      Camerapos::cam_height_meters, Camerapos::goal_height_meters, Camerapos::pitch,
    //      units::degree_t{result.GetBestTarget().GetPitch()}).value());
  }
}

// Called once the command ends or is interrupted.
void LockVisionTargetCommand::End(bool interrupted) {
  wpi::outs() << "Lock End\n";
}

// Returns true when the command should end.
bool LockVisionTargetCommand::IsFinished() {
  wpi::outs() << "Lock IsFinished\n";
  return false;
}
