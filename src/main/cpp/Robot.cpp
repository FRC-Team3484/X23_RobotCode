// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "constants.h"
#include "FRC3484_Lib/utils/SC_Functions.h"

#include "frc/PneumaticsModuleType.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/CommandScheduler.h>
#include "ctre/phoenix/motorcontrol/Faults.h"

#include "networktables/NetworkTableInstance.h"

using namespace frc;
using namespace SC;
using namespace ctre;
void Robot::RobotInit() 
{
	GP1_Driver = new XboxController(/*USB Port*/ 0);
	BB_GameDevice = new GenericHID(/*USB Port*/ 1);
	Gyroscope = new phoenix::sensors::Pigeon2(30);
  	_drivetrain = new X23_Drivetrain(std::make_tuple<int, int>(C_FX_FL_MASTER, C_FX_FL_SLAVE),
                                   std::make_tuple<int, int>(C_FX_FR_MASTER, C_FX_FR_SLAVE),
								   std::make_tuple<int, int>(C_FX_BL_MASTER, C_FX_BL_SLAVE),
                                   std::make_tuple<int, int>(C_FX_BR_MASTER, C_FX_BR_SLAVE),
                                   SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::REVPH, C_DRIVE_SOL});
}
/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
	frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() 
{
	/*======================*/
	/*====Driver Controls===*/
	/*======================*/

	if(GP1_Driver->GetRightBumper())
	{
		// Fine control mode; Scales driver input to smaller range for finer control
		Y_Demand = F_Scale(-100.0, 100.0, Throttle_Range_Fine, -GP1_Driver->GetLeftY());
		X_Demand = F_Scale(-100.0, 100.0, Throttle_Range_Fine, GP1_Driver->GetLeftX());
		Z_Demand = F_Scale(-100.0, 100.0, Throttle_Range_Fine, GP1_Driver->GetRightX());
	}
	else
	{
		// Normal control mode
		Y_Demand = F_Scale(-100.0, 100.0, Throttle_Range_Normal, -GP1_Driver->GetLeftY());
		X_Demand = F_Scale(-100.0, 100.0, Throttle_Range_Normal, GP1_Driver->GetLeftX());
		Z_Demand = F_Scale(-100.0, 100.0, Throttle_Range_Normal, GP1_Driver->GetRightX());
		// Y_Demand = -GP1_Driver->GetLeftY();
		// X_Demand = GP1_Driver->GetLeftX();
	}

	drivetrain_mode = GP1_Driver->GetAButton() || GP1_Driver->GetRightBumper();

	if(Gyroscope != nullptr)
	{
		_drivetrain->Drive(SC::F_Deadband(X_Demand, C_DRIVE_DEADBAND),
					 SC::F_Deadband(Y_Demand, C_DRIVE_DEADBAND), 
					 SC::F_Deadband(Z_Demand, C_DRIVE_DEADBAND), 
					 Gyroscope->GetYaw(),
					 drivetrain_mode);
	}
	else
	{
		_drivetrain->Drive(SC::F_Deadband(X_Demand, C_DRIVE_DEADBAND),
					 SC::F_Deadband(Y_Demand, C_DRIVE_DEADBAND), 
					 SC::F_Deadband(Z_Demand, C_DRIVE_DEADBAND), 
					 0.0,
					 drivetrain_mode);
	}
  
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
