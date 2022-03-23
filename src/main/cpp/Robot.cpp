// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <photonlib/PhotonUtils.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
  frc::SmartDashboard::PutNumber("auto_slot", 0);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

//  #ifdef SWERVE_SUBSYSTEM
//   frc::SmartDashboard::PutNumber("GyroAngle", (double)m_container.GetDriveSubsystem()->GetHeading());
//  #endif
//  #ifdef HOPPER_SUBSYSTEM
//  int ballColor = m_container.GetHopperSubsystem()->GetColor();
//  #else 
//  int ballColor = 0;
//  #endif
//  for (int i = 0; i < kBallStatusLength; i++) {
//        //Set the value
//        if(ballColor == 1) {
//          m_ballStatusLedBuffer[i].SetRGB(0,0,255);
//        } else if(ballColor == 0) {
//          m_ballStatusLedBuffer[i].SetRGB(255,0,0);
//        } else {
//          m_ballStatusLedBuffer[i].SetRGB(0,0,0);
//        }
//  }


    //m_ballStatusLed.SetLength(kBallStatusLength);
    //m_ballStatusLed.SetData(m_ballStatusLedBuffer);
    //m_ballStatusLed.Start();

}
/*
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }

  #ifdef HOPPER_SUBSYSTEM
  m_container.GetHopperSubsystem()->SetOverride(false);
  #endif

  #ifdef INTAKE_SUBSYSTEM
  m_container.GetIntakeSubsystem()->SetOverride(false);
  #endif
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
