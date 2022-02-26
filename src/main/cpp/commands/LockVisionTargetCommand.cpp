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
    photonlib::PhotonTrackedTarget target = result.GetBestTarget();
    wpi::outs() << "dist estamate\n"; 
    wpi::outs() << std::to_string(photonlib::PhotonUtils::CalculateDistanceToTarget(
          Camerapos::cam_height_meters, Camerapos::goal_height_meters, Camerapos::pitch,
          units::degree_t{result.GetBestTarget().GetPitch()}).value()) << "\n";
    frc::SmartDashboard::PutNumber("Yaw", target.GetYaw());
    frc::SmartDashboard::PutNumber("Pitch", target.GetPitch());
    frc::SmartDashboard::PutNumber("Skew", target.GetSkew());
    frc::SmartDashboard::PutNumber("Distance", photonlib::PhotonUtils::CalculateDistanceToTarget(
          Camerapos::cam_height_meters, Camerapos::goal_height_meters, Camerapos::pitch,
          units::degree_t{result.GetBestTarget().GetPitch()}).value());
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
