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
    slot_two_first = PathPlanner::loadPath("slot2 first", AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    slot_three_second = PathPlanner::loadPath("slot3 second", AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
   test_path = PathPlanner::loadPath("New Path", AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);

   //slot_two_first = PathPlanner::loadPath("slot2 first", 4_mps, 4_mps_sq);
   //slot_three_second = PathPlanner::loadPath("slot3 second", 4_mps, 4_mps_sq);
   //test_path = PathPlanner::loadPath("New Path", 4_mps, 4_mps_sq);
}

PathPlannerTrajectory* trajectories::get_auto_trajectory() {
    //std::cout << "GetNumber start\n";
    double slot = frc::SmartDashboard::GetNumber("auto_slot", 0);
    if (slot == 2) {
        return &slot_two_first;
    } else if(slot == 4) {
        return &test_path;
    }
    return &slot_three_second;

}