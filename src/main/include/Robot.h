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
#include "FRC3484_Lib/components/SC_OperatorInput.h"

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
  SC::SC_OperatorInput	*BB_GameDevice;
//  frc::GenericHID      *BB_GameDevice;  


MecanumAutoBuilder autoBuilder(
    [this]() { return swerveSubsystem.getPose(); }, // Function to supply current robot pose
    [this](auto initPose) { swerveSubsystem.resetPose(initPose); }, // Function used to reset odometry at the beginning of auto
    PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    [this](auto speeds) { swerveSubsystem.driveFieldRelative(speeds); }, // Output function that accepts field relative ChassisSpeeds
    eventMap, // Our event map
    { &swerveSubsystem }, // Drive requirements, usually just a single drive subsystem
    true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
);

};