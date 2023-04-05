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
#include "frc2/command/FunctionalCommand.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include <fmt/format.h>

RobotContainer::RobotContainer() {
  // Setup Auto Routines
  //for (auto autoConfig : autoNames) {
    //fmt::print("{}\n", std::string(autoConfig));

    // Add to chooser
    //autonomousChooser.AddOption(autoConfig, NoAutoCommand.get());
    //autonomousChooser.AddOption(autoConfig, MobilityAutoCommand.get());
    //autonomousChooser.AddOption(autoConfig, BalanceAutoCommand.get());
  //}

  autonomousChooser.AddOption("Hybrid/Mobility", MobilityAutoCommand.get());
  autonomousChooser.AddOption("Hybrid/Mobility/Balance", BalanceAutoCommand.get());
  autonomousChooser.AddOption("Cone 1/Mobility/Balance", ConeMIDBalanceAutoCommand.get());
	// //BAALLAANNCCIINNGG WOOOOOOOOOO(more auto ballance code here later maybe)
	// if(!auto2Done){
	// 	while(X23._drivetrain.GetDistance() > 6.0) 
	// 	{
	// 	X23._drivetrain.Drive(0, -0.25, 0, false, true);
	// 	}
	// 	auto2Done = true;
	// }
	//X23._drivetrain.Drive(0,0, 0, false, true);});
  // Default value and send to ShuffleBoard
  
    autonomousChooser.SetDefaultOption("NO AUTO", NoAutoCommand.get());

    frc::Shuffleboard::GetTab("X23").Add("Auto Chooser", autonomousChooser).WithWidget("ComboBox Chooser");
        this->DebounceSocks = new frc::Debouncer(2.0_s, frc::Debouncer::DebounceType::kRising);
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
  currentAuto = autonomousChooser.GetSelected();
  currentAuto->Schedule();

}

void RobotContainer::endAutoCommand() {
  // Cancel selected auto
  currentAuto->Cancel();
}
