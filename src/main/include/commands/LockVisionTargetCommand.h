// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSubsystem.h"
#include <frc/controller/HolonomicDriveController.h>
#include <photonlib/PhotonUtils.h>
#include <photonlib/PhotonCamera.h>
#include <wpi/span.h>
#include <units/length.h>
#include <math.h> 


#ifndef EXCLUDE_PATHPLANNER


#endif

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class LockVisionTargetCommand
    : public frc2::CommandHelper<frc2::CommandBase, LockVisionTargetCommand> {
 public:
   LockVisionTargetCommand(photonlib::PhotonCamera* camera);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  ShooterSubsystem *m_shooter;
  photonlib::PhotonCamera* m_camera;
};
