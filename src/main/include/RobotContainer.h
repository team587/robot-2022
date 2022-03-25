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
#include <frc2/command/WaitCommand.h>

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
#include <frc/AddressableLED.h>
#include <photonlib/PhotonCamera.h>
#include <frc/DigitalInput.h>

//Comment out the below line if deploying code for mini-bot.


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
#ifdef CMBJ

#endif
#ifdef SWERVE_SUBSYSTEM
  DriveSubsystem *GetDriveSubsystem() { return &m_drive; }
#endif

#ifdef HOPPER_SUBSYSTEM
  HopperSubsystem *GetHopperSubsystem() { return &m_hopperSubsystem;}
#endif

#ifdef INTAKE_SUBSYSTEM
  IntakeSubsystem *GetIntakeSubsystem() { return &m_intakeSubsystem;}
#endif

  frc2::Command* GetAutonomousCommand();

 private:
  photonlib::PhotonCamera m_camera;
  
  static constexpr int kLength = 62; // number of leds in rings
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;
  // Must be a PWM header, not MXP or DIO
  frc::AddressableLED m_led{0};

#ifdef CLIMBER_SUBSYSTEM
    
    rev::CANSparkMax m_climberMotor;
    frc::DigitalInput m_extendedDigitalInput;
    frc::DigitalInput m_contractedDigitalInput;
    ClimberSubsystem m_climberSubsystem;
    
#endif

#ifdef INTAKE_SUBSYSTEM

    //rev::CANSparkMax m_intakeMotor;//{canIDs::kIntakeMotor};
    //frc::Solenoid m_intakeSolenoid;//{frc::PneumaticsModuleType::CTREPCM, solenoidIDs::kIntakeSolenoid};
    IntakeSubsystem m_intakeSubsystem;//{&m_intakeMotor, &m_intakeSolenoid};
  
#endif

#ifdef SHOOTER_SUBSYSTEM
    ShooterSubsystem m_shooterSubsystem;

#endif

#ifdef HOPPER_SUBSYSTEM

    rev::CANSparkMax m_hopperMotor;
    HopperSubsystem m_hopperSubsystem;

    
#endif


  // The driver's controller
  frc::Joystick m_driverController{OIConstants::kDriverControllerPort};
  frc::Joystick m_coDriverController{OIConstants::kCoDriverControllerPort};
  

  // The robot's subsystems and commands are defined here..
  
  // The robot's subsystems
  #ifdef SWERVE_SUBSYSTEM
  DriveSubsystem m_drive;
  #endif

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();

#ifdef SWERVE_SUBSYSTEM
  frc2::InstantCommand m_ZeroHeading{[this] {m_drive.ZeroHeading(); }, {&m_drive}};
  frc2::InstantCommand m_setSpeedLow{[this] {m_drive.SetSpeedController(4.0); }, {&m_drive}};
  frc2::InstantCommand m_setSpeedMid{[this] {m_drive.SetSpeedController(2.0); }, {&m_drive}};
  frc2::InstantCommand m_setSpeedHigh{[this] {m_drive.SetSpeedController(1.0); }, {&m_drive}};
#endif


#ifdef INTAKE_SUBSYSTEM

  frc2::InstantCommand m_intakeDeploy{[this] {m_intakeSubsystem.Deploy(); }, {&m_intakeSubsystem}};
  frc2::InstantCommand m_intakeRetreat{[this] {m_intakeSubsystem.Retreat(); }, {&m_intakeSubsystem}};
  frc2::InstantCommand m_intakeToggle{[this] {m_intakeSubsystem.Toggle(); }, {&m_intakeSubsystem}};

#endif

#ifdef HOPPER_SUBSYSTEM 

  frc2::InstantCommand m_fireShooterOn{[this] {m_hopperSubsystem.setOverride(true); }, {&m_hopperSubsystem}};
  frc2::InstantCommand m_fireShooterOff{[this] {m_hopperSubsystem.setOverride(false); }, {&m_hopperSubsystem}};
  frc::DigitalInput m_hopperBallDetection;
  frc::DigitalInput m_uptakeBallDetection;
  frc2::InstantCommand m_reverseHopper{[this] {m_hopperSubsystem.ToggleReverse(); }, {&m_hopperSubsystem}};

#endif

#ifdef SHOOTER_SUBSYSTEM

  CycleHoodPositions m_hoodCycleUp{&m_shooterSubsystem, true};
  CycleHoodPositions m_hoodCycleDown{&m_shooterSubsystem, false};
  CycleTurretPositions m_turretCycleLeft{&m_shooterSubsystem, true};
  CycleTurretPositions m_turretCycleRight{&m_shooterSubsystem, false};
  frc2::InstantCommand m_stopShooter{[this] {m_shooterSubsystem.Stop(); }, {&m_shooterSubsystem}};

#endif

#ifdef SHOOTER_SUBSYSTEM

  frc2::InstantCommand m_shooterSpeed{[this] {m_shooterSubsystem.Start(); }, {&m_shooterSubsystem}};
  frc2::InstantCommand m_shooterOff{[this] {m_shooterSubsystem.Stop(); }, {&m_shooterSubsystem}};
  frc2::InstantCommand m_cycleShooterSpeed{[this] {m_shooterSubsystem.SpeedCycle(); }, {&m_shooterSubsystem}};
  frc2::InstantCommand m_adjustHoodAngle{[this] {m_shooterSubsystem.setHoodAngle(20); }, {&m_shooterSubsystem}}; 
  frc2::InstantCommand m_adjustHoodAngle0{[this] {m_shooterSubsystem.setHoodAngle(0); }, {&m_shooterSubsystem}}; 
  frc2::InstantCommand m_adjustTurretAngle{[this] {m_shooterSubsystem.setTurretAngle(90); }, {&m_shooterSubsystem}};
  
#endif

#ifdef SWERVE_SUBSYSTEM
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
#endif



#ifdef INTAKE_SUBSYSTEM
 //frc2::InstantCommand m_intakeSpeed{[this] {m_intakeSubsystem.IntakeSpeed(1); }, {&m_intakeSubsystem}};
 frc2::InstantCommand m_intakeSpeedOn{[this] {m_intakeSubsystem.setOverride(true); }, {&m_intakeSubsystem}};
 frc2::InstantCommand m_intakeSpeedOff{[this] {m_intakeSubsystem.setOverride(false); }, {&m_intakeSubsystem}};
#endif
#ifdef SWERVE_SUBSYSTEM
  frc2::SequentialCommandGroup m_slotCommand1 {
    #ifdef COMPETITIONBOT
    m_shooterSpeed,
    m_intakeDeploy,
    m_intakeSpeedOn,
    m_adjustHoodAngle,
    m_adjustTurretAngle,
    frc2::WaitCommand{units::second_t(2)},   
    m_fireShooterOn,
    frc2::WaitCommand{units::second_t(1)},
    m_adjustHoodAngle0,
    #endif
    m_autoCommand1_0,
    m_stopDriving,
    m_fireShooterOn,
    frc2::WaitCommand{units::second_t(5)},
    m_fireShooterOff,
    m_shooterOff, 
    m_intakeSpeedOff,
    //m_autoCommand1_1,
    //m_stopDriving,
    #ifdef HOPPER_SUBSYSTEM
   // m_fireShooterOn,
  
    //m_intakeSpeedOff,
    //m_fireShooterOff,
    //m_shooterOff,
    #endif

    //m_autoCommand1_2,
    //m_stopDriving
  };

  frc2::SequentialCommandGroup m_slotCommand2 {
    #ifdef COMPETITIONBOT
    m_shooterSpeed,
    m_intakeDeploy,
    m_intakeSpeedOn,
    m_adjustHoodAngle,
    m_adjustTurretAngle,
    frc2::WaitCommand{units::second_t(2)},   
    m_fireShooterOn,
    frc2::WaitCommand{units::second_t(1)},
    m_adjustHoodAngle0,
    m_autoCommand1_0,
    m_stopDriving,
    m_fireShooterOn,
    frc2::WaitCommand{units::second_t(5)},
    m_fireShooterOff,
    m_shooterOff, 
    m_intakeSpeedOff,
    m_autoCommand1_1,
    m_stopDriving,
    #endif
  };

  frc2::SequentialCommandGroup m_slotCommand3 {
    #ifdef COMPETITIONBOT
    m_shooterSpeed,
    m_intakeDeploy,
    m_intakeSpeedOn,
    m_adjustHoodAngle,
    m_adjustTurretAngle,
    #endif
    m_autoCommand3_0,
    m_stopDriving,
    #ifdef HOPPER_SUBSYSTEM
    m_fireShooterOn,
    frc2::WaitCommand{units::second_t(2)},
    m_intakeSpeedOff,
    m_fireShooterOff,
    m_shooterOff,
    #endif
    //m_autoCommand3_1,
    //m_autoCommand3_2,
    //m_stopDriving
  };

  frc2::SequentialCommandGroup m_slotCommand4 {
    #ifdef COMPETITIONBOT
    m_shooterSpeed,
    m_intakeDeploy,
    m_intakeSpeedOn,
    m_adjustHoodAngle,
    m_adjustTurretAngle,
    #endif
    m_autoCommand4_0,
    m_stopDriving,
    #ifdef HOPPER_SUBSYSTEM
    m_fireShooterOn,
    frc2::WaitCommand{units::second_t(2)},
    m_intakeSpeedOff,
    m_fireShooterOff,
    m_shooterOff,
    #endif
    //m_autoCommand4_1,
    //m_autoCommand4_2,
    //m_stopDriving
  };
  #endif
};
