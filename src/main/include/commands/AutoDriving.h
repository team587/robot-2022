// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"
#include <frc/controller/HolonomicDriveController.h>
#include <frc/Timer.h>
#include "Trajectories.h"

#ifndef EXCLUDE_PATHPLANNER

using namespace pathplanner;

#endif

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoDriving
    : public frc2::CommandHelper<frc2::CommandBase, AutoDriving> {
 public:
  explicit AutoDriving(DriveSubsystem* subsystem, int slot, int numPath);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

#ifndef EXCLUDE_PATHPLANNER

    PathPlannerTrajectory* Trajectory;

#endif 

    DriveSubsystem *m_driveSubsystem;
    int m_slot;
    int m_numPath;
    double lastTime;
    bool ignoreHeading;
    
    trajectories m_trajectory;
    frc::Timer m_timer;
    

    frc::HolonomicDriveController controller{
       frc2::PIDController{1, 0, 0}, frc2::PIDController{1, 0, 0},
       frc::ProfiledPIDController<units::radian>{1, 0, 0, frc::TrapezoidProfile<units::radian>::Constraints{6.28_rad_per_s, 3.14_rad_per_s / 1_s}}
    };


};
