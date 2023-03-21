#include "X23_RobotContainer.h"

// #include <iostream>

#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
// #include <frc/trajectory/Trajectory.h>
// #include <frc/trajectory/TrajectoryConfig.h>
// #include <frc/trajectory/TrajectoryGenerator.h>
// #include <frc/trajectory/TrajectoryUtil.h>

#include <frc2/command/RunCommand.h>

#include "frc2/command/CommandScheduler.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include <fmt/format.h>

RobotContainer::RobotContainer() {
  // Setup Auto Routines
 /* for (auto autoConfig : autoNames) {
    fmt::print("{}\n", std::string(autoConfig));
    // Build Commands
    autoCommands.emplace_back(
        autoBuilder.fullAuto(pathplanner::PathPlanner::loadPathGroup(
            autoConfig, 1_mps, 10_mps_sq, false)));

    // Add to chooser
    autonomousChooser.AddOption(autoConfig, autoCommands.back().get());
  }
 // autoCommands.emplace_back(autonomousChooser.AddOption(noAutoCommand));
  // Default value and send to ShuffleBoard
  */
  //autonomousChooser.SetDefaultOption("no_auto", noAutoCommand.get());

  //frc::Shuffleboard::GetTab("X23").Add("Auto Chooser", autonomousChooser).WithWidget("ComboBox Chooser");
}

void RobotContainer::setAutoDefaults() {
  // By default leave the drivetrain stopped so the robot doesnt move if
  // something goes wrong. No other defaults are given so the manuiulator and
  // claw retain thier state during auto.
  //_drivetrain.SetDefaultCommand(
  //    frc2::RunCommand([this]() { _drivetrain.StopMotors(); }));
}

void RobotContainer::startAutoCommand() {
  // Get command from chooser to scheduel  
  //currentAuto = autonomousChooser.GetSelected();
  //currentAuto->Schedule();

}

void RobotContainer::endAutoCommand() {
  // Cancel selected auto
  //currentAuto->Cancel();
}
