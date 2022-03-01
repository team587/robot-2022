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
    wpi::span<photonlib::PhotonTrackedTarget> goalresults;
    int y = 0;
    for(int x = targets.size(); x>0; x--){
      if(abs(targets[x].GetPitch()-meanPitch)>10.0){
        goalresults[y] = targets[x];
        y++;
      }
      if(y==5){
        break;
      }
    };

    //Distance 1 calculation, gets the distance based off of the spacing between tape peices
    //finds the mean skew
    double mean_yaw = 0;
    for(int x = 0; x<goalresults.size(); x++){
      mean_yaw+=goalresults[x].GetYaw();
    }
    mean_yaw/=goalresults.size();


    //sorts by abs(yaw)
    for(int x = 0; x<goalresults.size(); x++){
      for(int i=0; i<goalresults.size()-1; i++){
        if(abs(goalresults[x].GetYaw()-mean_yaw)>abs(goalresults[x+1].GetYaw()-mean_yaw)){
          auto temp = goalresults[x+1];
          goalresults[x+1] = goalresults[x];
          goalresults[x] = temp;
        }
      }
    }

    //Actualy calculates the distance
    if(goalresults.size()>2){
      double yaw_between = (abs(goalresults[1].GetYaw()-goalresults[0].GetYaw())+abs(goalresults[2].GetYaw()-goalresults[0].GetYaw()))/2.0;
      double dist = atan(yaw_between)/Camerapos::tape_spacing.value();
    }

    //Does other calculations
    wpi::outs() << "dist estamate\n"; 
    photonlib::PhotonTrackedTarget target = result.GetBestTarget();
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
