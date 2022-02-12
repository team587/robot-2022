// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "Trajectories.h"
#include "commands/AutoDriving.h"

//Comment out the below line if deploying code for mini-bot.
#define COMPETITIONBOT

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  DriveSubsystem *GetDriveSubsystem() { return &m_drive; }



  frc2::Command* GetAutonomousCommand();

 private:

    
#ifdef COMPETITIONBOT
    
    rev::CANSparkMax m_climberMotor;
    frc::DigitalInput m_extendedDigitalInput;
    frc::DigitalInput m_contractedDigitalInput;
    ClimberSubsystem m_climberSubsystem;
    
    WPI_TalonSRX m_intakeMotor;//{canIDs::kIntakeMotor};
    frc::Solenoid m_intakeSolenoid;//{frc::PneumaticsModuleType::CTREPCM, solenoidIDs::kIntakeSolenoid};
    IntakeSubsystem m_intakeSubsystem;//{&m_intakeMotor, &m_intakeSolenoid};
  

    rev::CANSparkMax m_shooterMotor1;
    rev::CANSparkMax m_shooterMotor2;
    rev::CANSparkMax m_hoodMotor;
    rev::CANSparkMax m_turningMotor;
    ShooterSubsystem m_shooterSubsystem;
    
#endif

  // The driver's controller
  frc::Joystick m_driverController{OIConstants::kDriverControllerPort};
  frc::Joystick m_coDriverController{OIConstants::kCoDriverControllerPort};
  

  // The robot's subsystems and commands are defined here..
  
  // The robot's subsystems
  DriveSubsystem m_drive;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();


  frc2::InstantCommand m_ZeroHeading{[this] {m_drive.ZeroHeading(); }, {&m_drive}};
  
  #ifdef COMPETITIONBOT
  frc2::InstantCommand m_zeroIntakeDeploy{[this] {m_intakeSubsystem.Deploy(); }, {&m_intakeSubsystem}};
  frc2::InstantCommand m_zeroIntakeRetreat{[this] {m_intakeSubsystem.Retreat(); }, {&m_intakeSubsystem}};
  #endif

  AutoDriving m_autoCommand1;
  AutoDriving m_autoCommand2;
  AutoDriving m_autoCommand3;
  AutoDriving m_autoCommand4;

  frc2::SequentialCommandGroup m_slotCommand1 {
    m_autoCommand1
  };

  frc2::SequentialCommandGroup m_slotCommand2 {
    m_autoCommand2
  };

  frc2::SequentialCommandGroup m_slotCommand3 {
    m_autoCommand3
  };

  frc2::SequentialCommandGroup m_slotCommand4 {
    m_autoCommand4
  };
  
};
