// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDriving.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/CommandBase.h>
#include <iostream>

AutoDriving::AutoDriving(DriveSubsystem* subsystem, int slot, int numPath) : 
    m_driveSubsystem(subsystem), 
    m_slot(slot), 
    m_numPath(numPath),
    m_initialY(0.0) {
  std::cout << "Constructor Header\n";
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(subsystem);
  std::cout << "Constructor Exit\n";
}

// Called when the command is initially scheduled.
void AutoDriving::Initialize() {
  std::cout << "Initialize\n";

 // m_driveSubsystem->ZeroHeading();
 omegaPidController.EnableContinuousInput((units::radian_t)0.0, (units::radian_t)wpi::numbers::pi);
#ifndef EXCLUDE_PATHPLANNER

  Trajectory = m_trajectory.get_auto_trajectory(m_slot, m_numPath);
  PathPlannerTrajectory::PathPlannerState *initial_state = Trajectory->getInitialState();

  m_initialY = (double)initial_state->pose.Y();
  //frc::Pose2d tmpPose{initial_state->pose.X(), (units::meter_t)8.23 - initial_state->pose.Y(), initial_state->pose.Rotation()};
  //initial_state->pose = tmpPose;
  m_driveSubsystem->ResetOdometry(initial_state->pose);
  omegaPidController.Reset(initial_state->pose.Rotation().Radians());
  
#endif

  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void AutoDriving::Execute() {

#ifndef EXCLUDE_PATHPLANNER

  units::time::second_t time = m_timer.Get();
  PathPlannerTrajectory::PathPlannerState state = Trajectory->sample(time);

  double tmpY = (double)state.pose.Y();
  tmpY = -1.0 * (tmpY - m_initialY);
  tmpY = tmpY + m_initialY;

  frc::Pose2d tmpPose{state.pose.X(), (units::meter_t)tmpY, state.pose.Rotation()};
  state.pose = tmpPose;
  //frc::Pose2d tmpPose{state.pose.X(), (units::meter_t)8.23 - state.pose.Y(), state.pose.Rotation()};
  //state.pose = tmpPose;

  frc::Rotation2d rotPi((units::radian_t)wpi::numbers::pi);
  state.holonomicRotation.RotateBy(rotPi); 
  
  std::cout << (double)state.pose.X() << " pose x ";
  std::cout << (double)state.pose.Y() << " pose y ";
  std::cout << (double)state.pose.Rotation().Radians() << " pose rot ";
  std::cout << (double)state.holonomicRotation.Radians() << " holonomic ";
  std::cout << (double)m_driveSubsystem->GetHeading() << " heading \n";

  frc::Pose2d RobotPose = m_driveSubsystem->GetPose();
  std::cout << (double)RobotPose.X() << " robot x ";
  std::cout << (double)RobotPose.Y() << " robot y ";
  std::cout << (double)RobotPose.Rotation().Radians() << " robot rot "; 
  //const auto adjustedSpeeds = controller.Calculate(RobotPose, state.pose, state.velocity, state.holonomicRotation);
  double vx = xPidController.Calculate((double)RobotPose.X(), (double)state.pose.X());
  double vy = yPidController.Calculate((double)RobotPose.Y(), (double)state.pose.Y());
  double vomega = omegaPidController.Calculate((units::radian_t)m_driveSubsystem->GetHeading(), (units::radian_t)state.holonomicRotation.Radians());
  //double vomega = 0.0;

  vx *= 3.0;
  vy *= 3.0;
  std::cout << vx << " x ";
  std::cout << vy << " y ";
  std::cout << vomega << " omega \n";

/*
  std::cout << (double)adjustedSpeeds.vx << " x ";
  std::cout << (double)adjustedSpeeds.vy << " y ";
  std::cout << (double)adjustedSpeeds.omega << " omega \n";

  auto states = m_driveSubsystem->kDriveKinematics.ToSwerveModuleStates( 
          frc::ChassisSpeeds::FromFieldRelativeSpeeds( adjustedSpeeds.vx, 
                                                       adjustedSpeeds.vy, 
                                                       adjustedSpeeds.omega, 
                                                       frc::Rotation2d(units::radian_t(m_driveSubsystem->GetHeading()))));
*/ 
   auto states = m_driveSubsystem->kDriveKinematics.ToSwerveModuleStates( 
          frc::ChassisSpeeds::FromFieldRelativeSpeeds( (units::meters_per_second_t)vx, 
                                                       (units::meters_per_second_t)vy, 
                                                       (units::radians_per_second_t)vomega, 
                                                       frc::Rotation2d(units::radian_t(m_driveSubsystem->GetHeading()))));
 m_driveSubsystem->SetModuleStates(states);

  auto [fl, fr, bl, br] = states;
  std::cout << (double)fl.speed << " fls ";
  std::cout << (double)fr.speed << " frs ";

  std::cout << (double)bl.speed << " bls ";
  std::cout << (double)br.speed << " brs ";

  std::cout << (double)fl.angle.Radians() << " fla ";
  std::cout << (double)fr.angle.Radians() << " fra ";

  std::cout << (double)bl.angle.Radians() << " bla ";
  std::cout << (double)br.angle.Radians() << " bra \n";


 #endif 

}

// Called once the command ends or is interrupted.
void AutoDriving::End(bool interrupted) {
  std::cout << "Auto Drive finished !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
  m_timer.Stop();
  m_driveSubsystem->Drive(units::meters_per_second_t(0), units::meters_per_second_t(0), units::radians_per_second_t(0), true);
}

// Returns true when the command should end.
bool AutoDriving::IsFinished() {
  //std::cout << "IsFinished\n";

#ifndef EXCLUDE_PATHPLANNER

  return m_timer.HasElapsed(Trajectory->getTotalTime());

#else

  return true;

#endif
}
