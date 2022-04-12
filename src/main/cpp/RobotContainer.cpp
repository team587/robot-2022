// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <rev/CANSparkMax.h>

#include "commands/AutoDriving.h"
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/SwerveModule.h"
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>


using namespace DriveConstants;

RobotContainer::RobotContainer():
 
    
    
#ifdef CLIMBER_SUBSYSTEM  
        m_climberMotor {canIDs::kClimberMotorPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
        m_extendedDigitalInput {canIDs::kExtendedDigitalInput},
        m_contractedDigitalInput {canIDs::kContractedDigitalInput},
        m_climberSubsystem {&m_climberMotor, &m_extendedDigitalInput, &m_contractedDigitalInput},
        
#endif

#ifdef INTAKE_SUBSYSTEM
        //m_intakeMotor {canIDs::kIntakeMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
        //m_intakeSolenoid {frc::PneumaticsModuleType::CTREPCM, solenoidIDs::kIntakeSolenoid}, 
        //m_intakeSubsystem {&m_intakeMotor, &m_intakeSolenoid},
        
#endif



#ifdef HOPPER_SUBSYSTEM

        m_hopperMotor {canIDs::kHopperMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
        m_hopperSubsystem {&m_hopperMotor, &m_shooterSubsystem, &m_intakeSubsystem, &m_hopperBallDetection, &m_uptakeBallDetection},
        m_hopperBallDetection {hopperBallDetectionPort},
        m_uptakeBallDetection {uptakeBallDetectionPort},

#endif


#ifdef SWERVE_SUBSYSTEM

    
    m_autoCommand1_0(&m_drive, 1, 0),
    m_autoCommand1_1(&m_drive, 1, 1),
    m_autoCommand1_2(&m_drive, 1, 2),
    
    m_autoCommand2_0(&m_drive, 2, 0),
    m_autoCommand2_1(&m_drive, 2, 1),
    m_autoCommand2_2(&m_drive, 2, 2),

    m_autoCommand3_0(&m_drive, 3, 0),
    m_autoCommand3_1(&m_drive, 3, 1),
    m_autoCommand3_2(&m_drive, 3, 2),

    m_autoCommand4_0(&m_drive, 4, 0),
    m_autoCommand4_1(&m_drive, 4, 1),


    m_autoCommand4_2(&m_drive, 4, 2)
#endif
{

    for (int i = 0; i < kLength; i++) {
        //Set the value
        m_ledBuffer[i].SetRGB(0,255,0);
    }
    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();

#ifdef SWERVE_SUBSYSTEM
    m_chooser.SetDefaultOption("Slot 1", &m_slotCommand1);
    m_chooser.AddOption("Slot 2", &m_slotCommand2);
    m_chooser.AddOption("Slot 3", &m_slotCommand3);
    m_chooser.AddOption("Slot 4", &m_slotCommand4);
#endif

    frc::SmartDashboard::PutData(&m_chooser);
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  #ifdef SWERVE_SUBSYSTEM
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            units::meters_per_second_t(m_driverController.GetRawAxis(xLeftJoystickVertical)),
            units::meters_per_second_t(m_driverController.GetRawAxis(xLeftJoystickHorizontal)),
            units::radians_per_second_t(2.0*m_driverController.GetRawAxis(xRightJoystickHorizontal)), true);
            m_drive.SetSpeedController(m_driverController.GetRawAxis(xRightTrigger));

            
            /*
            units::meters_per_second_t(m_driverController.GetRawAxis(xLeftJoystickVertical)),
            units::meters_per_second_t(m_driverController.GetRawAxis(xLeftJoystickHorizontal)),
            units::radians_per_second_t(2.0*m_driverController.GetRawAxis(xRightJoystickHorizontal)), true);
            */
      },
      {&m_drive}));
  #endif
 

#ifdef INTAKE_SUBSYSTEM
 m_intakeSubsystem.SetDefaultCommand(frc2::RunCommand(
      [this] {
       //m_intakeSubsystem.IntakeSpeed(m_coDriverController.GetRawAxis(leftJoystickVertical));
       m_intakeSubsystem.IntakeSpeed(m_coDriverController.GetRawAxis(xLeftJoystickVertical));
      },
      {&m_intakeSubsystem}));
#endif
  
}

void RobotContainer::ConfigureButtonBindings() {



//These are the drive controllers

#ifdef SHOOTER_SUBSYSTEM 

    frc2::Button{[&] {return m_driverController.GetRawButton(xButtonY);}}.WhenPressed(&m_LowerPortSpeed);

#endif

#ifdef SWERVE_SUBSYSTEM
    frc2::Button{[&] {return m_driverController.GetRawButton(xButtonMenu);}}.WhenPressed(&m_ZeroHeading);


     
    //frc2::Button{[&] {return m_driverController.GetRawButton(xRightTrigger);}}.WhenPressed(&m_setSpeedLow);
    //frc2::Button{[&] {return m_driverController.GetRawButton(xRightTrigger);}}.WhenReleased(&m_setSpeedHigh);
    frc2::Button{[&] {return m_driverController.GetRawButton(xRightBumper);}}.WhenPressed(&m_setSpeedMid);
    frc2::Button{[&] {return m_driverController.GetRawButton(xRightBumper);}}.WhenReleased(&m_setSpeedHigh);

    //frc2::Button{[&] {return m_driverController.GetRawButton(xButtonMenu);}}.WhenPressed(&m_ZeroHeading);
    //frc2::Button{[&] {return m_driverController.GetRawButton(xRightTrigger);}}.WhenPressed(&m_setSpeedLow); //this was our mistake - Beck
    //frc2::Button{[&] {return m_driverController.GetRawButton(xRightTrigger);}}.WhenReleased(&m_setSpeedHigh);
    //frc2::Button{[&] {return m_driverController.GetRawButton(xRightBumper);}}.WhenPressed(&m_setSpeedMid);
    //frc2::Button{[&] {return m_driverController.GetRawButton(xRightBumper);}}.WhenReleased(&m_setSpeedHigh);
#endif


    //frc2::JoystickButton(&m_driverController, buttonA).WhenPressed(m_LockVisionTargetCommand);
    //frc2::JoystickButton(&m_driverController, buttonA).WhenPressed(frc2::PrintCommand("Testing"));


#ifdef INTAKE_SUBSYSTEM

//These are also drive controllers

    frc2::Button{[&] {return m_driverController.GetRawButton(xButtonX);}}.WhenPressed(&m_intakeToggle);
    //frc2::Button{[&] {return m_driverController.GetRawButton(xButtonX);}}.WhenPressed(&m_intakeToggle);

   // frc2::Button{[&] {return m_driverController.GetRawButton(buttonY);}}.WhenPressed(&m_intakeRetreat);

//These are the co-driver controllers
   //frc2::Button{[&] {return m_coDriverController.GetRawButton(buttonX);}}.WhenPressed(&m_intakeToggle);
    frc2::Button{[&] {return m_coDriverController.GetRawButton(xButtonX);}}.WhenPressed(&m_intakeToggle);
   // frc2::Button{[&] {return m_coDriverController.GetRawButton(buttonY);}}.WhenPressed(&m_intakeRetreat);

#endif

#ifdef HOPPER_SUBSYSTEM

    //frc2::Button{[&] {return m_driverController.GetRawButton(buttonY);}}.WhenPressed(&m_reverseHopper);
    //frc2::Button{[&] {return m_coDriverController.GetRawButton(buttonY);}}.WhenPressed(&m_reverseHopper);

    //frc2::Button{[&] {return m_driverController.GetRawButton(buttonB);}}.WhenPressed(&m_fireShooter);
    //frc2::Button{[&] {return m_coDriverController.GetRawButton(buttonB);}}.WhenPressed(&m_fireShooter);

#endif
#ifdef SHOOTER_SUBSYSTEM

   /*frc2::Button{[&] {return m_coDriverController.GetRawButton(rightBumper);}}.WhenPressed(&m_hoodCycleUp);
    frc2::Button{[&] {return m_coDriverController.GetRawButton(leftBumper);}}.WhenPressed(&m_hoodCycleDown);
    frc2::Button{[&] {return m_coDriverController.GetRawButton(leftTrigger);}}.WhenPressed(&m_turretCycleLeft);
    frc2::Button{[&] {return m_coDriverController.GetRawButton(rightTrigger);}}.WhenPressed(&m_turretCycleRight);
    frc2::Button{[&] {return m_coDriverController.GetRawButton(buttonA);}}.WhenPressed(&m_cycleShooterSpeed);
    //frc2::Button{[&] {return m_coDriverController.GetRawButton(buttonB);}}.WhenPressed(&m_shooterOff);
    frc2::Button{[&] {return m_coDriverController.GetRawButton(buttonB);}}.WhenPressed(&m_stopShooter);
   */

    //frc2::Button{[&] {return m_coDriverController.GetRawButton(xRightBumper);}}.WhenPressed(&m_hoodCycleUp);
    //frc2::Button{[&] {return m_coDriverController.GetRawButton(xLeftBumper);}}.WhenPressed(&m_hoodCycleDown);
    frc2::Button{[&] {return m_coDriverController.GetRawButton(xRightBumper);}}.WhenPressed(&m_turretCycleLeft);
    frc2::Button{[&] {return m_coDriverController.GetRawButton(xLeftBumper);}}.WhenPressed(&m_turretCycleRight);
    frc2::Button{[&] {return m_coDriverController.GetRawButton(xButtonA);}}.WhenPressed(&m_cycleShooterSpeed);
    frc2::Button{[&] {return m_coDriverController.GetRawButton(xButtonB);}}.WhenPressed(&m_stopShooter);
#endif
    frc2::Button{[&] {return m_coDriverController.GetRawButton(xButtonMenu);}}.WhenPressed(&m_toggleClimb);
}



frc2::Command* RobotContainer::GetAutonomousCommand() {
   return m_chooser.GetSelected();
    //return new AutoDriving(this);
}

/*
frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                        units::radian_t(wpi::numbers::pi));

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() {
            m_drive.Drive(units::meters_per_second_t(0),
                          units::meters_per_second_t(0),
                          units::radians_per_second_t(0), false);
          },
          {}));
}
*/