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
    slot1 = PathPlanner::loadPath("slot1", AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    slot2 = PathPlanner::loadPath("slot2", AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    slot3 = PathPlanner::loadPath("slot3", AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    slot4 = PathPlanner::loadPath("slot4", AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);

   //slot_two_first = PathPlanner::loadPath("slot2 first", 4_mps, 4_mps_sq);
   //slot_three_second = PathPlanner::loadPath("slot3 second", 4_mps, 4_mps_sq);
   //test_path = PathPlanner::loadPath("New Path", 4_mps, 4_mps_sq);
}

PathPlannerTrajectory* trajectories::get_auto_trajectory(int slot) {
    //std::cout << "GetNumber start\n";
    //double slot = frc::SmartDashboard::GetNumber("auto_slot", 0);
    if (slot ==1 ) {
        return &slot1;
    } else if (slot == 2) {
        return &slot2;
    } else if (slot == 3) {
        return &slot3;
    } else if (slot == 4) {
        return &slot4;
    }
    return &slot1;

}