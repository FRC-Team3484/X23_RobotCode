// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "constants.h"

#include "frc/PneumaticsModuleType.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/CommandScheduler.h>
#include "ctre/phoenix/motorcontrol/Faults.h"

#include "networktables/NetworkTableInstance.h"

using namespace frc;

void Robot::RobotInit() 
{
  	GP1_driver = new XboxController(C_DRIVER_USB);

	_drivetrain = new X23_Drivetrain();

}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
}
void Robot::TeleopPeriodic() {

#ifdef DRIVE_MODE_ARCADE

}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
#endif

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
