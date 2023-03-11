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
  	GP2_Driver = new SC::SC_OperatorInput(C_OPERATOR_USB);
  	// _drivetrain = new X23_Drivetrain(std::make_tuple<int, int>(C_FX_FL_MASTER, C_FX_FL_SLAVE),
    //                                std::make_tuple<int, int>(C_FX_FR_MASTER, C_FX_FR_SLAVE),
	// 							   std::make_tuple<int, int>(C_FX_BL_MASTER, C_FX_BL_SLAVE),
    //                                std::make_tuple<int, int>(C_FX_BR_MASTER, C_FX_BR_SLAVE),
    //                                SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::CTREPCM, C_DRIVE_SOL},
	// 							   C_PIGEON_IMU); 

	// _intake = new X23_Intake(C_SPX_INTAKE_LEFT, C_SPX_INTAKE_RIGHT);
	// _elevator = new X23_Elevator(C_FX_ELEVATEMOTOR,
	// 							C_FX_TILTMOTOR,
	// 							SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::CTREPCM, C_SOL_CLAW_GRIP},
	// 							SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::CTREPCM, C_SOL_CLAW_TILT},
	// 							SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::CTREPCM, C_SOL_ELEVATOR_BRAKE},
	// 							C_DI_CH_ELEVATOR_TILT_HOME,
	// 							C_DI_CH_ELEVATOR_HOME, 
	// 							C_DI_CH_ELEVATOR_TILT_MAX);


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
	X23.startAutoCommand();
}

void Robot::AutonomousPeriodic() {}
void Robot::AutonomousExit()
{
	X23.endAutoCommand();
}
void Robot::TeleopInit() 
{
	if(GP2_Driver != nullptr)
	{
		//intake controls on the button box
		GP2_Driver->GetRawButton(C_GD_COLLECT_CONE_LEFT).OnTrue(X23._intake.Collect_ConeLeft());
		GP2_Driver->GetRawButton(C_GD_COLLECT_CONE_LEFT).OnFalse(X23._intake.StopIntake());

		GP2_Driver->GetRawButton(C_GD_COLLECT_CONE_RIGHT).OnTrue(X23._intake.Collect_ConeRight());
		GP2_Driver->GetRawButton(C_GD_COLLECT_CONE_RIGHT).OnFalse(X23._intake.StopIntake());

		GP2_Driver->GetRawButton(C_GD_COLLECT_CUBE_OR_CONECENTER).OnTrue(X23._intake.Collect_Cube_Or_ConeCenter());
		GP2_Driver->GetRawButton(C_GD_COLLECT_CUBE_OR_CONECENTER).OnFalse(X23._intake.StopIntake());

		GP2_Driver->GetRawButton(C_GD_COLLECT_EJECT).OnTrue(X23._intake.Collect_Eject());
		GP2_Driver->GetRawButton(C_GD_COLLECT_EJECT).OnFalse(X23._intake.StopIntake());
		//claw controls on the button box
		GP2_Driver->GetRawButton(C_GD_CLAW_GRAB).OnTrue(X23._elevator.ToggleClawOpen());
		GP2_Driver->GetRawButton(C_GD_CLAW_GRAB).OnFalse(X23._elevator.ToggleClawShut());
		GP2_Driver->GetRawButton(C_GD_CLAW_TILT).OnTrue(X23._elevator.ClawTilt());
		GP2_Driver->GetRawButton(C_GD_CLAW_TILT).OnFalse(X23._elevator.StopTilt());
		//elevator controls on the button box
		GP2_Driver->GetRawButton(C_GD_ELE_CUBEMID).OnTrue(X23._elevator.CubeOne());
		GP2_Driver->GetRawButton(C_GD_ELE_CUBEHI).OnTrue(X23._elevator.CubeTwo());

		GP2_Driver->GetRawButton(C_GD_ELE_CONEMID).OnTrue(X23._elevator.ConeOne());
		GP2_Driver->GetRawButton(C_GD_ELE_CONEHI).OnTrue(X23._elevator.ConeTwo());

		GP2_Driver->GetRawButton(C_GD_ELE_UNIVERSAL).OnTrue(X23._elevator.HybridZone());
		GP2_Driver->GetRawButton(C_GD_ELE_HOME).OnTrue(X23._elevator.HomePOS());
		GP2_Driver->GetRawButton(C_GD_ELE_FEEDER).OnTrue(X23._elevator.Substation());
	}
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() 
{

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


	X23._drivetrain.Drive(SC::F_Deadband(X_Demand, C_DRIVE_DEADBAND),
							SC::F_Deadband(Y_Demand, C_DRIVE_DEADBAND), 
							SC::F_Deadband(Z_Demand, C_DRIVE_DEADBAND), 
							true,
							drivetrain_mode);
	/*==========================*/
	/*===Game Device Controls===*/
	/*==========================*/
	if(X23._elevator.pidDisabled()){
		X23._elevator.ControlDirectElevate(F_Deadband(GP2_Driver->GetAxis(C_GD_J1_ELE_HIGHT),0.05));
		X23._elevator.ControlDirectTilt(F_Deadband(GP2_Driver->GetAxis(C_GD_J2_ELE_ANGLE),0.05));
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
