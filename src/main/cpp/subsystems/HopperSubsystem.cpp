/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//#include <frc/Joystick.h>
//#include <frc/Relay.h>
#include <Constants.h>

#include "subsystems/HopperSubsystem.h"
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <iostream>
#include <string>
#include <frc/shuffleboard/shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include "subsystems/ShooterSubsystem.h"
#include <frc/DriverStation.h>
#include <subsystems/IntakeSubsystem.h>


HopperSubsystem::HopperSubsystem(
    rev::CANSparkMax *hopperMotor,
    rev::CANSparkMax *uptakeMotor,
    ShooterSubsystem *shooterSub, 
    IntakeSubsystem *intakeSub,
    frc::DigitalInput *hopperBallDetection,
    frc::DigitalInput *uptakeBallDetection) {
  m_hopperMotor = hopperMotor;
  m_uptakeMotor = uptakeMotor;
  m_shooterSub = shooterSub;
  m_intakeSub = intakeSub;
  m_hopperBallDetection = hopperBallDetection;
  m_uptakeBallDetection = uptakeBallDetection;
  m_hopperSpeed = -0.3;
  m_autoOverride = false;


  //hopperSpeed = 0;
  SetUptakeSpeed(0);
  SetHopperSpeed(0);

  //frc::Shuffleboard::GetTab("Hopper").Add ("Hopper speed", hopperSpeed);

  //frc::SmartDashboard::PutNumber("Hopper Speed", hopperSpeed);
} 
/*
void HopperSubsystem::HopperStart(){
  m_hopperMotor->Set(hopperSpeed);
}
void HopperSubsystem::HopperReverse(){
  m_hopperMotor->Set(hopperSpeed * -1);
}
void HopperSubsystem::HopperStop(){
  m_hopperMotor->Set(0);
}*/

void HopperSubsystem::Periodic() {
  
 // m_reversed = m_coDriverController.GetRawButton(buttonY);

  // Implementation of subsystem periodic method goes here.
  //hopperSpeed = frc::SmartDashboard::GetNumber("Hopper Speed", hopperSpeed);
  //hopperSpeed = frc::Shuffleboard::GetTab("Hopper")
  //bool detectBall = m_detectBall.Get();
  /*int currentColor = GetColor();
  frc::DriverStation::Alliance alliance = frc::DriverStation::GetAlliance();

  if (frc::DriverStation::Alliance::kBlue == alliance && currentColor == 0) {
    m_shooterSub->SetDumpMode(true);
  } else if (frc::DriverStation::Alliance::kRed == alliance && currentColor == 1) {
    m_shooterSub->SetDumpMode(true);
  } else {
    m_shooterSub->SetDumpMode(false);
  }*/
 //
 //turn these in button bindings...these don't seem right????
 //
 //setLoadingSpeed(m_coDriverController.GetRawAxis(-1.0 * xLeftTrigger));
 //setLoadingSpeed(m_coDriverController.GetRawAxis(xRightTrigger));

  /*if(autoOverride) {
    setLoadingSpeed(1);
  } else {
    if (m_DriverController.GetRawButton(buttonB) *//*|| m_coDriverController.GetRawButton(buttonB)*///) {
      /*setLoadingSpeed(m_reversed ? -1.0 : 1.0);
    } else {
      setLoadingSpeed(m_reversed ? -1.0 : 0.0);
    }
  }*/
  
  
  //Ball = currentColor != -1;
  /*bool ball = !m_uptakeBallDetection->Get();
  bool index = !m_hopperBallDetection->Get();
  bool deploy = m_intakeSub->getDeployed();

  if  
  ((!ball && !index && !deploy) || (ball && index && !deploy) || (ball && index && deploy)) {
    m_hopperMotor->Set(m_reversed ? -1 * m_hopperSpeed : 0);
  } else {
    m_hopperMotor->Set(m_reversed ? -1 * m_hopperSpeed : m_hopperSpeed);
  }*/
  //if (m_coDriverController.GetRawButton(leftJoystickButton)) {
  //  double joystickAxis = m_coDriverController.GetRawAxis(leftJoystickVertical);
  //  m_hopperMotor->Set(joystickAxis);
  /*} else if ((currentColor == 0 || currentColor == 1) && detectBall == true) {
    m_hopperMotor->Set(0);
  } else {
    m_hopperMotor->Set(hopperSpeed);
    //setLoadingSpeed(0);
  }*/
  //frc::SmartDashboard::PutString("Detected Color", ConvertColor(currentColor));
  //frc::SmartDashboard::PutBoolean("HopperBall", Index);
  //frc::SmartDashboard::PutBoolean("UptakeBall", Ball);
}
/*
int HopperSubsystem::GetColor(){
  frc::Color detectedColor = m_colorSensor.GetColor();
  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  double tolerance = frc::SmartDashboard::GetNumber("Tolerance", .9);
  frc::SmartDashboard::PutNumber("Tolerance", tolerance);
  rev::ColorMatch Matcher;
  for (int x = 0; x < 2; x++){
  Matcher.AddColorMatch(kColorCodes[x]);
  }
  Matcher.SetConfidenceThreshold(tolerance);
std::optional<frc::Color> matchedColor = Matcher.MatchColor(detectedColor); 
//frc::SmartDashboard::PutNumber("Matched Red", matchedColor.has_value() ? matchedColor.value().red : 0);
//frc::SmartDashboard::PutNumber("Matched Blue", matchedColor.has_value() ? matchedColor.value().blue : 0);
//frc::SmartDashboard::PutNumber("Matched Green", matchedColor.has_value() ? matchedColor.value().green : 0);
  int colorIndex = -1;
  for (int color = 0; color < 2; color++) {
    if (matchedColor == kColorCodes[color]) {
      //frc::SmartDashboard::PutNumber("Color", color);
      colorIndex = color;
    }
  }
  return colorIndex;
}

std::string HopperSubsystem::ConvertColor(int colorIndex){
  if (colorIndex == 0) {
    return "R";
  } else if (colorIndex == 1) {
    return "B";
  } else {
    return "Unknown";
  }
}
*/

void HopperSubsystem::SetHopperSpeed(double speed) {
  m_hopperMotor->Set(speed);
  //should we always run uptake and hopper at the same time???
}

void HopperSubsystem::SetUptakeSpeed(double speed) {
  m_uptakeMotor->Set(-speed);
}


