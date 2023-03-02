// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "constants.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "Subsystems/X23_Intake.h"
#include "frc/PneumaticsModuleType.h"
#include <pathplanner/lib/auto/MecanumAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>
#include "frc2/command/Commands.h"
#include "frc2/command/CommandScheduler.h"

using namespace frc;
using namespace frc2;
using namespace SC;
using namespace ctre;
using namespace pathplanner;

void Robot::RobotInit() 
{
	GP1_Driver = new XboxController(/*USB Port*/ C_DRIVER_USB);
	BB_GameDevice = new SC_OperatorInput(/*USB Port*/ 1);
  	_drivetrain = new X23_Drivetrain(std::make_tuple<int, int>(C_FX_FL_MASTER, C_FX_FL_SLAVE),
                                   std::make_tuple<int, int>(C_FX_FR_MASTER, C_FX_FR_SLAVE),
								   std::make_tuple<int, int>(C_FX_BL_MASTER, C_FX_BL_SLAVE),
                                   std::make_tuple<int, int>(C_FX_BR_MASTER, C_FX_BR_SLAVE),
                                   SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::CTREPCM, C_DRIVE_SOL},
								   C_PIGEON_IMU); 

	_intake = new X23_Intake(C_SPX_INTAKE_LEFT, C_SPX_INTAKE_RIGHT);
	_elevator = new X23_Elevator(C_FX_ELEVATEMOTOR,
								C_FX_TILTMOTOR,
								SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::CTREPCM, C_SOL_CLAW_GRIP},
								SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::CTREPCM, C_SOL_CLAW_TILT},
								SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::CTREPCM, C_SOL_ELEVATOR_BRAKE},
								C_DI_CH_ELEVATOR_TILT_HOME,
								C_DI_CH_ELEVATOR_HOME, 
								C_DI_CH_ELEVATOR_TILT_MAX);


	Throttle_Range_Normal(-C_DRIVE_MAX_DEMAND, C_DRIVE_MAX_DEMAND);
	Throttle_Range_Fine(-C_DRIVE_MAX_DEMAND_FINE, C_DRIVE_MAX_DEMAND_FINE);

	// FunEvents.emplace("ElevatorHybrid", std::make_shared<Cmd_Elev_HybridZone>(_elevator));
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
void Robot::AutonomousInit() 
{
// 	this ->autoBuilder = new MecanumAutoBuilder
// (
//     [this]() { return _drivetrain->GetPose(); }, // Function to supply current robot pose
//     [this](auto initPose) { _drivetrain->SetPose(initPose); }, // Function used to reset odometry at the beginning of auto
//     PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
//     PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
//     [this](auto speeds) { _drivetrain->DriveAuto(speeds); }, // Output function that accepts field relative ChassisSpeeds
//     FunEvents, // Our event map
//     std::initializer_list<frc2::Subsystem*>( _drivetrain ), // Drive requirements, usually just a single drive subsystem
//     true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
// );
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() 
{
	if(BB_GameDevice != nullptr)
	{
		//intake controls on the button box
		BB_GameDevice->GetButton(C_GD_COLLECT_CONE_LEFT).OnTrue(frc2::cmd::RunOnce([this] { this->_intake->Collect_ConeLeft(); }));
		BB_GameDevice->GetButton(C_GD_COLLECT_CONE_LEFT).OnFalse(frc2::cmd::RunOnce([this] { this->_intake->StopIntake(); }));

		BB_GameDevice->GetButton(C_GD_COLLECT_CONE_RIGHT).OnTrue(frc2::cmd::RunOnce([this] { this->_intake->Collect_ConeRight(); }));
		BB_GameDevice->GetButton(C_GD_COLLECT_CONE_RIGHT).OnFalse(frc2::cmd::RunOnce([this] { this->_intake->StopIntake(); }));

		BB_GameDevice->GetButton(C_GD_COLLECT_CUBE_OR_CONECENTER).OnTrue(frc2::cmd::RunOnce([this] { this->_intake->Collect_Cube_Or_ConeCenter(); }));
		BB_GameDevice->GetButton(C_GD_COLLECT_CUBE_OR_CONECENTER).OnFalse(frc2::cmd::RunOnce([this] { this->_intake->StopIntake(); }));

		BB_GameDevice->GetButton(C_GD_COLLECT_EJECT).OnTrue(frc2::cmd::RunOnce([this] { this->_intake->Collect_Eject(); }));
		BB_GameDevice->GetButton(C_GD_COLLECT_EJECT).OnFalse(frc2::cmd::RunOnce([this] { this->_intake->StopIntake(); }));
		//claw controls on the button box
		BB_GameDevice->GetButton(C_GD_CLAW_GRAB).OnTrue(frc2::cmd::RunOnce([this]{ this->_elevator->ToggleClaw(); }));
		BB_GameDevice->GetButton(C_GD_CLAW_TILT).OnTrue(frc2::cmd::RunOnce([this]{ this->_elevator->ClawTilt(); }));
		BB_GameDevice->GetButton(C_GD_CLAW_TILT).OnFalse(frc2::cmd::RunOnce([this]{ this->_elevator->StopTilt(); }));
		//elevator controls on the button box
		BB_GameDevice->GetButton(C_GD_ELE_CUBEMID).OnTrue(frc2::cmd::RunOnce([this]{ this->_elevator->CubeOne(); }));
		BB_GameDevice->GetButton(C_GD_ELE_CUBEHI).OnTrue(frc2::cmd::RunOnce([this]{ this->_elevator->CubeTwo(); }));

		BB_GameDevice->GetButton(C_GD_ELE_CONEMID).OnTrue(frc2::cmd::RunOnce([this]{ this->_elevator->ConeOne(); }));
		BB_GameDevice->GetButton(C_GD_ELE_CONEHI).OnTrue(frc2::cmd::RunOnce([this]{ this->_elevator->ConeTwo(); }));

		BB_GameDevice->GetButton(C_GD_ELE_UNIVERSAL).OnTrue(frc2::cmd::RunOnce([this]{ this->_elevator->HybridZone(); }));
		BB_GameDevice->GetButton(C_GD_ELE_HOME).OnTrue(frc2::cmd::RunOnce([this]{ this->_elevator->HomePOS(); }));
		BB_GameDevice->GetButton(C_GD_ELE_FEEDER).OnTrue(frc2::cmd::RunOnce([this]{ this->_elevator->Substation(); }));
	}
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() 
{
   double encVal = 0, error = 0;

	/*======================*/
	/*====Driver Controls===*/
	/*======================*/

	if(GP1_Driver->GetLeftBumper())
	{
		// Fine control mode; Scales driver input to smaller range for finer control
		Y_Demand = F_Scale(-1.0, 1.0, Throttle_Range_Fine, -GP1_Driver->GetLeftY());
		X_Demand = F_Scale(-1.0, 1.0, Throttle_Range_Fine, GP1_Driver->GetLeftX());
		Z_Demand = F_Scale(-1.0, 1.0, Throttle_Range_Fine, GP1_Driver->GetRightX());
	}
	else
	{
		// Normal control mode
		Y_Demand = F_Limit(Throttle_Range_Normal, GP1_Driver->GetLeftY());
		X_Demand = F_Limit(Throttle_Range_Normal, GP1_Driver->GetLeftX());
		Z_Demand = F_Limit(Throttle_Range_Normal, GP1_Driver->GetRightX());
	}

	drivetrain_mode = GP1_Driver->GetRawButton(XBOX_RB);


		_drivetrain->Drive(SC::F_Deadband(X_Demand, C_DRIVE_DEADBAND),
					 SC::F_Deadband(Y_Demand, C_DRIVE_DEADBAND), 
					 SC::F_Deadband(Z_Demand, C_DRIVE_DEADBAND), 
					 true,
					 drivetrain_mode);
	/*==========================*/
	/*===Game Device Controls===*/
	/*==========================*/

	
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
