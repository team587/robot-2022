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

    WPI_TalonSRX m_intakeMotor;
    frc::Solenoid m_intakeSolenoid;  

    IntakeSubsystem m_intakeSubsystem;
  
    rev::CANSparkMax m_climberMotor;
    frc::DigitalInput m_extendedDigitalInput;
    frc::DigitalInput m_contractedDigitalInput;

    ClimberSubsystem m_climberSubsystem;

    rev::CANSparkMax m_shooterMotor1;
    rev::CANSparkMax m_shooterMotor2;
    rev::CANSparkMax m_hoodMotor;
    rev::CANSparkMax m_turningMotor;

    ShooterSubsystem m_shooterSubsystem;

  // The driver's controller
  frc::Joystick m_driverController{OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
  frc2::InstantCommand m_ZeroHeading{[this] {m_drive.ZeroHeading(); }, {&m_drive}};
};
