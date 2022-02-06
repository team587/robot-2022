// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "trajectories.h"
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include "Constants.h"

using namespace pathplanner;

trajectories::trajectories() {
    std::cout << "Trajectories start\n";
    //slot_two_first = PathPlanner::loadPath("slot2 first", AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    //slot_three_second = PathPlanner::loadPath("slot3 second", AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);

    slot_two_first = PathPlanner::loadPath("slot2 first", 8_mps, 5_mps_sq);
    slot_three_second = PathPlanner::loadPath("slot3 second", 8_mps, 5_mps_sq);
    std::cout << "Trajectories end\n";
}

PathPlannerTrajectory* trajectories::get_auto_trajectory() {
    std::cout << "GetNumber start\n";
    double slot = frc::SmartDashboard::GetNumber("auto_slot", 0);
    if (slot == 2) {
        std::cout << "slot_two_first\n";
        return &slot_two_first;
    }
    std::cout << "slot_three_second\n";
    return &slot_three_second;

}