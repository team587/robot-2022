// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>

#pragma once

#define COMPETITIONBOT
//#define EXCLUDE_PATHPLANNER
#define CLIMBER_SUBSYSTEM
#define HOPPER_SUBSYSTEM
#define INTAKE_SUBSYSTEM
#define SHOOTER_SUBSYSTEM
#define TURRET_SUBSYSTEM
#define SWERVE_SUBSYSTEM
#define CMBJ


/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

enum DigitalInputs {
    hopperBallDetectionPort = 0,
    uptakeBallDetectionPort = 1,
};

enum JoystickButtonConstants {
    buttonX = 1,
    buttonA = 2,
    buttonB = 3,
    buttonY = 4,
    leftBumper = 5,
    rightBumper = 6,
    leftTrigger = 7,
    rightTrigger = 8,
    buttonBack = 9,
    buttonStart = 10,
    leftJoystickButton = 11,
    rightJoystickButton = 12
};

enum XBoxJoystickButtonConstants {
    xButtonA = 1,
    xButtonB = 2,
    xButtonX = 3,
    xButtonY = 4,
    xLeftBumper = 5,
    xRightBumper = 6,
    xButtonView = 7,
    xButtonMenu = 8,
    xLeftJoystickButton = 9,
    xRightJoystickButton = 10
};

enum JoystickAxesConstants {
    leftJoystickHorizontal = 0,
    leftJoystickVertical = 1,
    rightJoystickHorizontal = 2,
    rightJoystickVertical = 3
};

enum XboxJoystickAxesConstants {
    xLeftJoystickHorizontal = 0,
    xLeftJoystickVertical = 1,
    xLeftTrigger = 2,
    xRightTrigger = 3,
    xRightJoystickHorizontal = 4,
    xRightJoystickVertical = 5

};

namespace DriveConstants {
constexpr int kFrontLeftDriveMotorPort = 6;
constexpr int kRearLeftDriveMotorPort = 8;
constexpr int kFrontRightDriveMotorPort = 3;
constexpr int kRearRightDriveMotorPort = 1;

constexpr int kFrontLeftTurningMotorPort = 5;
constexpr int kRearLeftTurningMotorPort = 7;
constexpr int kFrontRightTurningMotorPort = 4;
constexpr int kRearRightTurningMotorPort = 2;

constexpr int kFrontLeftAbsoluteEncoderPort = 23;
constexpr int kFrontRightAbsoluteEncoderPort = 22;
constexpr int kRearLeftAbsoluteEncoderPort = 24;
constexpr int kRearRightAbsoluteEncoderPort = 21;


constexpr bool kFrontLeftTurningEncoderReversed = true;
constexpr bool kRearLeftTurningEncoderReversed = true;
constexpr bool kFrontRightTurningEncoderReversed = true;
constexpr bool kRearRightTurningEncoderReversed = true;


constexpr bool kFrontLeftDriveEncoderReversed = false;
constexpr bool kRearLeftDriveEncoderReversed = false;
constexpr bool kFrontRightDriveEncoderReversed = false;
constexpr bool kRearRightDriveEncoderReversed = true;

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The SysId tool provides a convenient
// method for obtaining these values for your robot.
constexpr auto ks = 1_V;
constexpr auto kv = 0.8 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.15 * 1_V * 1_s * 1_s / 1_m;

// Example value only - as above, this must be tuned for your drive!
constexpr double kPFrontLeftVel = 0.5;
constexpr double kPRearLeftVel = 0.5;
constexpr double kPFrontRightVel = 0.5;
constexpr double kPRearRightVel = 0.5;
}  // namespace DriveConstants

namespace ModuleConstants {
constexpr int kEncoderCPR = 1024;
constexpr double kWheelDiameterMeters = 0.15;
constexpr double kDriveEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterMeters * wpi::numbers::pi) /
    static_cast<double>(kEncoderCPR);

constexpr double kTurningEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (wpi::numbers::pi * 2) / static_cast<double>(kEncoderCPR);

constexpr double kPModuleTurningController = 1;
constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

namespace AutoConstants {
using radians_per_second_squared_t =
    units::compound_unit<units::radians,
                         units::inverse<units::squared<units::second>>>;

constexpr auto kMaxSpeed = units::meters_per_second_t(4);
constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(4);
constexpr auto kMaxAngularSpeed = units::radians_per_second_t(3.142);
constexpr auto kMaxAngularAcceleration =
    units::unit_t<radians_per_second_squared_t>(3.142);

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

//

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kCoDriverControllerPort = 1;
}  // namespace OIConstants

enum solenoidIDs {
    kIntakeSolenoid = 0,
    kIntakeSolenoid1 = 1
};

enum canIDs {
    kIntakeMotor = 17,
    kClimberMotorPort = 10,
    kExtendedDigitalInput = 11,
    kContractedDigitalInput = 12,
    kShooterMotor1 = 13,
    kShooterMotor2 = 14,
    kHoodMotor = 15,
    kTurningMotor = 16,
    kHopperMotor = 9,
    kLoadShooterMotor =18
};
namespace Camerapos { //All of these save goal height are completly made up.
constexpr auto cam_height_meters = units::meter_t(.3937); // We will have to calculate this dynamicly, but this will be the base hight when the camera is at its lowest position. It is also possible that the camera is close enough to the center of the hood rotation such that it does not matter.
constexpr auto goal_height_meters = units::meter_t(1.59385);// This is 8ft in meters
constexpr auto tape_spacing = units::inch_t(10.5); //spacing between peices of tape
constexpr auto angle_offset = 109.05; 
constexpr auto shooter_max = 36.8;
}