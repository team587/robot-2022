// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "trajectories.h"
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace pathplanner;

trajectories::trajectories() {
    PathPlannerTrajectory slot_two_first = PathPlanner::loadPath("slot2 first", 8_mps, 5_mps_sq);
    PathPlannerTrajectory slot_three_second = PathPlanner::loadPath("slot3 second", 8_mps, 5_mps_sq);
}

PathPlannerTrajectory* trajectories::get_auto_trajectory() {
    double slot = frc::SmartDashboard::GetNumber("auto_slot", 0);
    if (slot == 2)
        return &slot_two_first;

    return &slot_three_second;
}