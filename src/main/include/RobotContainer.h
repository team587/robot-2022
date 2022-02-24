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
#include "commands/CycleHoodPositions.h"
#include "commands/CycleTurretPositions.h"
#include "subsystems/HopperSubsystem.h"
#include "commands/AdjustHoodAngle.h"
#include "commands/TurretAngle.h"

//Comment out the below line if deploying code for mini-bot.
//#define COMPETITIONBOT

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

    rev::CANSparkMax m_hopperMotor;
    HopperSubsystem m_hopperSubsystem;

    
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
  frc2::InstantCommand m_setSpeedLow{[this] {m_drive.SetSpeedController(4.0); }, {&m_drive}};
  frc2::InstantCommand m_setSpeedMid{[this] {m_drive.SetSpeedController(2.0); }, {&m_drive}};
  frc2::InstantCommand m_setSpeedHigh{[this] {m_drive.SetSpeedController(1.0); }, {&m_drive}};

  #ifdef COMPETITIONBOT
  frc2::InstantCommand m_zeroIntakeDeploy{[this] {m_intakeSubsystem.Deploy(); }, {&m_intakeSubsystem}};
  frc2::InstantCommand m_zeroIntakeRetreat{[this] {m_intakeSubsystem.Retreat(); }, {&m_intakeSubsystem}};
  CycleHoodPositions m_hoodCycleUp{&m_shooterSubsystem, true};
  CycleHoodPositions m_hoodCycleDown{&m_shooterSubsystem, false};

  CycleTurretPositions m_turretCycleLeft{&m_shooterSubsystem, true};
  CycleTurretPositions m_turretCycleRight{&m_shooterSubsystem, false};
  #endif

  AutoDriving m_autoCommand1_0;
  AutoDriving m_autoCommand1_1;
  AutoDriving m_autoCommand1_2;
  
  AutoDriving m_autoCommand2_0;
  AutoDriving m_autoCommand2_1;
  AutoDriving m_autoCommand2_2;
  
  AutoDriving m_autoCommand3_0;
  AutoDriving m_autoCommand3_1;
  AutoDriving m_autoCommand3_2;

  AutoDriving m_autoCommand4_0;
  AutoDriving m_autoCommand4_1;
  AutoDriving m_autoCommand4_2;
  
  frc2::InstantCommand m_stopDriving{[this] {m_drive.Drive(units::meters_per_second_t(0),
                          units::meters_per_second_t(0),
                          units::radians_per_second_t(0), false); }, {&m_drive}};

  #ifdef COMPETITIONBOT

  frc2::InstantCommand m_shooterSpeed{[this] {m_shooterSubsystem.Start(); }, {&m_shooterSubsystem}};
  frc2::InstantCommand m_intakeSpeed{[this] {m_intakeSubsystem.IntakeSpeed(1); }, {&m_intakeSubsystem}};
  AdjustHoodAngle m_adjustHoodAngle; //{25, &m_shooterSubsystem};
  TurretAngle m_turretAngle; //{90, &m_shooterSubsystem};

  #endif

  frc2::SequentialCommandGroup m_slotCommand1 {
    #ifdef COMPETITIONBOT
    m_shooterSpeed,
    m_zeroIntakeDeploy,
    m_intakeSpeed,
    m_adjustHoodAngle,
    m_turretAngle,
    #endif
    m_autoCommand1_0,
    m_stopDriving,
    m_autoCommand1_1,
    m_autoCommand1_2,
    m_stopDriving
  };

  frc2::SequentialCommandGroup m_slotCommand2 {
    #ifdef COMPETITIONBOT
    m_shooterSpeed,
    m_zeroIntakeDeploy,
    m_intakeSpeed,
    m_adjustHoodAngle,
    m_turretAngle,
    #endif
    m_autoCommand2_0,
    m_stopDriving,
    m_autoCommand2_1,
    m_autoCommand2_2, 
    m_stopDriving
  };

  frc2::SequentialCommandGroup m_slotCommand3 {
    #ifdef COMPETITIONBOT
    m_shooterSpeed,
    m_zeroIntakeDeploy,
    m_intakeSpeed,
    m_adjustHoodAngle,
    m_turretAngle,
    #endif
    m_autoCommand3_0,
    m_stopDriving,
    m_autoCommand3_1,
    m_autoCommand3_2,
    m_stopDriving
  };

  frc2::SequentialCommandGroup m_slotCommand4 {
    #ifdef COMPETITIONBOT
    m_shooterSpeed,
    m_zeroIntakeDeploy,
    m_intakeSpeed,
    m_adjustHoodAngle,
    m_turretAngle,
    #endif
    m_autoCommand4_0,
    m_stopDriving,
    m_autoCommand4_1,
    m_autoCommand4_2,
    m_stopDriving
  };
};
