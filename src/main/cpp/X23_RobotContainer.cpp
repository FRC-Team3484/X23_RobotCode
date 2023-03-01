#include "X23_RobotContainer.h"

#include <iostream>

#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
// #include <frc/trajectory/Trajectory.h>
// #include <frc/trajectory/TrajectoryConfig.h>
// #include <frc/trajectory/TrajectoryGenerator.h>
// #include <frc/trajectory/TrajectoryUtil.h>

#include <frc2/command/RunCommand.h>

#include <pathplanner/lib/PathPlanner.h>

#include "Subsystems/X23_Drivetrain.h"

#include "frc2/command/CommandScheduler.h"

#include "pathplanner/lib/PathConstraints.h"
#include "pathplanner/lib/PathPlannerTrajectory.h"
#include "pathplanner/lib/PathPoint.h"

#include "frc/smartdashboard/SmartDashboard.h"

RobotContainer::RobotContainer() {
  // Setup Auto Routines
  for (auto autoConfig : autoNames) {
    // Build Commands
    autoCommands.emplace_back(
        autoBuilder.fullAuto(pathplanner::PathPlanner::loadPathGroup(
            autoConfig, 1_mps, units::acceleration::meters_per_second_squared_t{10}, false)));

    // Add to chooser
    autonomousChooser.AddOption(autoConfig, autoCommands.back().get());
  }

  // Default value and send to ShuffleBoard
  autonomousChooser.SetDefaultOption("no_auto", noAutoCommand.get());
  frc::SmartDashboard::PutData("Auto Chooser", &autonomousChooser);
}

void RobotContainer::setAutoDefaults() {
  // By default leave the drivetrain stopped so the robot doesnt move if
  // something goes wrong. No other defaults are given so the manuiulator and
  // claw retain thier state during auto.
  _drivetrain.SetDefaultCommand(
      frc2::RunCommand([this]() { _drivetrain.StopMotors(); }));
}

void RobotContainer::startAutoCommand() {
  // Get command from chooser to scheduel
  currentAuto = autonomousChooser.GetSelected();
  currentAuto->Schedule();
}

void RobotContainer::endAutoCommand() {
  // Cancel selected auto
  currentAuto->Cancel();
}