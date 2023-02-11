// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "Constants.h"
#include "Subsystems/X23_Drivetrain.h"
#include "Subsystems/X23_Intake.h"
#include "Subsystems/X23_Elevator.h"
#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "FRC3484_Lib/utils/SC_Datatypes.h"

#include <frc/XboxController.h>

#include "ctre/phoenix/sensors/Pigeon2.h"


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.

  X23_Drivetrain *_drivetrain = nullptr;
  X23_Intake *_intake = nullptr;
  X23_Elevator *_elevator = nullptr;

  SC::SC_Range<double> Throttle_Range_Normal = {-C_DRIVE_MAX_DEMAND, C_DRIVE_MAX_DEMAND};
  SC::SC_Range<double> Throttle_Range_Fine = {-C_DRIVE_MAX_DEMAND_FINE, C_DRIVE_MAX_DEMAND_FINE};

  double X_Demand, Y_Demand, Z_Demand;
  double drivetrain_mode;

  frc::XboxController  *GP1_Driver;// GP = Gamepad
  frc::GenericHID      *BB_Jgerald; // Jgerald is the name of the button box, thank you jystin <3

  ctre::phoenix::sensors::Pigeon2 *Gyroscope;
};