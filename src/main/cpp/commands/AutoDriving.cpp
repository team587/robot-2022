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
    m_numPath(numPath) {
  std::cout << "Constructor Header\n";
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(subsystem);
  std::cout << "Constructor Exit\n";
}

// Called when the command is initially scheduled.
void AutoDriving::Initialize() {
  std::cout << "Initialize\n";

 // m_driveSubsystem->ZeroHeading();

#ifndef EXCLUDE_PATHPLANNER

  Trajectory = m_trajectory.get_auto_trajectory(m_slot, m_numPath);
  PathPlannerTrajectory::PathPlannerState *initial_state = Trajectory->getInitialState();
  m_driveSubsystem->ResetOdometry(initial_state->pose);
  
#endif

  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void AutoDriving::Execute() {

#ifndef EXCLUDE_PATHPLANNER

  units::time::second_t time = m_timer.Get();
  PathPlannerTrajectory::PathPlannerState state = Trajectory->sample(time);
  std::cout << (double)state.pose.X() << " pose x ";
  std::cout << (double)state.pose.Y() << " pose y ";
  std::cout << (double)state.pose.Rotation().Radians() << " pose rot ";
  std::cout << (double)state.holonomicRotation.Radians() << " holonomic ";
  std::cout << (double)m_driveSubsystem->GetHeading() << " heading \n";

  frc::Pose2d RobotPose = m_driveSubsystem->GetPose();
  std::cout << (double)RobotPose.X() << " robot x ";
  std::cout << (double)RobotPose.Y() << " robot y ";
  std::cout << (double)RobotPose.Rotation().Radians() << " robot rot "; 
  const auto adjustedSpeeds = controller.Calculate(RobotPose, state.pose, state.velocity, state.holonomicRotation);

  //frc::Rotation2d robotangle(m_driveSubsystem->GetHeading());
  //const auto adjustedSpeeds = controller.Calculate(RobotPose, state.pose, state.velocity, robotangle);
  std::cout << (double)adjustedSpeeds.vx << " x ";
  std::cout << (double)adjustedSpeeds.vy << " y ";
  std::cout << (double)adjustedSpeeds.omega << " omega \n";

  //m_driveSubsystem->Drive(adjustedSpeeds.vx/8.0, adjustedSpeeds.vy/8.0, adjustedSpeeds.omega, true);

  //auto [fl, fr, bl, br] = m_container->GetDriveSubsystem()->kDriveKinematics.ToSwerveModuleStates(adjustedSpeeds);
  m_driveSubsystem->SetModuleStates(m_driveSubsystem->kDriveKinematics.ToSwerveModuleStates(adjustedSpeeds));
  /*
  std::cout << (double)fl.speed << " fls ";
  std::cout << (double)fr.speed << " frs ";

  std::cout << (double)bl.speed << " bls ";
  std::cout << (double)br.speed << " brs ";

  std::cout << (double)fl.angle.Radians() << " fla ";
  std::cout << (double)fr.angle.Radians() << " fra ";

  std::cout << (double)bl.angle.Radians() << " bla ";
  std::cout << (double)br.angle.Radians() << " bra \n";
  */

 #endif 

}

// Called once the command ends or is interrupted.
void AutoDriving::End(bool interrupted) {
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
