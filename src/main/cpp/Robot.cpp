// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "constants.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "Subsystems/X23_Intake.h"
#include "frc/PneumaticsModuleType.h"
#include "frc2/command/Commands.h"
#include "frc2/command/CommandScheduler.h"

using namespace frc;
using namespace frc2;
using namespace SC;
using namespace ctre;

void Robot::RobotInit() 
{
	GP1_Driver = new PS4Controller(/*USB Port*/ C_DRIVER_USB);
  	GP2_Operator = new SC::SC_OperatorInput(C_OPERATOR_USB);
  	//GP2_Operator = new XboxController(C_OPERATOR_USB);

	Throttle_Range_High(-C_DRIVE_MAX_DEMAND_HIGH, C_DRIVE_MAX_DEMAND_HIGH);
	Throttle_Range_Normal(-C_DRIVE_MAX_DEMAND_MID, C_DRIVE_MAX_DEMAND_MID);
	Throttle_Range_Fine(-C_DRIVE_MAX_DEMAND_FINE, C_DRIVE_MAX_DEMAND_FINE);

	X23._elevator.InitNetworkTables();
}
/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
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
void Robot::AutonomousInit() 
{ 
	X23.startAutoCommand();
	//auto1Done = false;
	X23._drivetrain.ResetPose();
}

void Robot::AutonomousPeriodic() 
{
/*	//MESS WITH CONE LOL
	if(!auto1Done)
	{
		while(X23._drivetrain.GetDistance() < 13.0)
		{
			  
			X23._drivetrain.Drive(0, 0.25, 0, false, true);
		}
		
		auto1Done = true;
		X23._drivetrain.Drive(0,0, 0, false, true);
	}

	while(X23._drivetrain.GetDistance() > 6.0) 
	{
		X23._drivetrain.Drive(0, -0.25, 0, false, true);
	}
	
	X23._drivetrain.Drive(0,0, 0, false, true);
*/
}

void Robot::AutonomousExit()
{
	X23.endAutoCommand();
}
void Robot::TeleopInit() 
{
	//frc2::CommandScheduler::GetInstance().CancelAll();
X23._drivetrain.SetPose(180.0);
	if(GP2_Operator != nullptr)
	{
		//intake controls on the button box
		GP2_Operator->GetRawButton(C_GD_COLLECT_CUBE).OnTrue(X23._intake.Collect_Cube());
		GP2_Operator->GetRawButton(C_GD_COLLECT_CUBE).OnFalse(X23._intake.StopIntake());

		GP2_Operator->GetRawButton(C_GD_COLLECT_CONE).OnTrue(X23._intake.Collect_Cone());
		GP2_Operator->GetRawButton(C_GD_COLLECT_CONE).OnFalse(X23._intake.StopIntake());

		GP2_Operator->GetRawButton(C_GD_CONE_EJECT).OnTrue(X23._intake.Eject_Cone());
		GP2_Operator->GetRawButton(C_GD_CONE_EJECT).OnFalse(X23._intake.StopIntake());

		GP2_Operator->GetRawButton(C_GD_CUBE_EJECT).OnTrue(X23._intake.Eject_Cube());
		GP2_Operator->GetRawButton(C_GD_CUBE_EJECT).OnFalse(X23._intake.StopIntake());

		//claw controls on the button box
		//GP2_Operator->GetRawButton(C_GD_CUBE_EJECT).OnTrue(X23._elevator.ToggleClawOpen());
		//GP2_Operator->GetRawButton(C_GD_CUBE_EJECT).OnFalse(X23._elevator.ToggleClawShut());
		//GP2_Operator->GetRawButton(C_GD_FREE2).OnTrue(X23._elevator.ClawTilt());
		//GP2_Operator->GetRawButton(C_GD_FREE2).OnFalse(X23._elevator.StopTilt());

		//elevator controls on the button box
		GP2_Operator->GetRawButton(C_GD_ELE_CUBEMID).OnTrue(X23._elevator.CubeOne());
		GP2_Operator->GetRawButton(C_GD_ELE_CUBEHI).OnTrue(X23._elevator.CubeTwo());

		GP2_Operator->GetRawButton(C_GD_ELE_CONEMID).OnTrue(X23._elevator.ConeOne());
		GP2_Operator->GetRawButton(C_GD_ELE_CONEHI).OnTrue(X23._elevator.ConeTwo());

		GP2_Operator->GetRawButton(C_GD_ELE_CubeHybrid).OnTrue(X23._elevator.HybridZoneCone());
		GP2_Operator->GetRawButton(C_GD_ELE_ConeHybrid).OnTrue(X23._elevator.HybridZoneCube());
		GP2_Operator->GetRawButton(C_GD_ELE_HOME).OnTrue(X23._elevator.HomePOS());
		GP2_Operator->GetRawButton(C_GD_PICKUP).OnTrue(X23._elevator.PickupPOS());
		GP2_Operator->GetRawButton(C_GD_ELE_FEEDER).OnTrue(X23._elevator.Substation());
		GP2_Operator->GetRawButton(7).OnTrue(X23._elevator.StopMotors());
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
	if(GP1_Driver->GetRawButtonPressed(C_DRIVER_GYRO_RESET_BTN)) { X23._drivetrain.ResetGyro(); }


    if(GP1_Driver->GetRawButton(C_DRIVER_FINE_ADJ_BTN))
    {
        // Fine control mode; Scales driver input to smaller range for finer control
        Y_Demand = F_Scale(-1.0, 1.0, Throttle_Range_Fine, GP1_Driver->GetRawAxis(C_DRIVER_FWD_REV_AXIS)); //->GetLeftY());
        X_Demand = F_Scale(-1.0, 1.0, Throttle_Range_Fine, GP1_Driver->GetRawAxis(C_DRIVER_LFT_RHT_AXIS)); //->GetLeftX());
        Z_Demand = F_Scale(-1.0, 1.0, Throttle_Range_Fine, GP1_Driver->GetRawAxis(C_DRIVER_ROTATE_AXIS)); //->GetRightX());
    }
    else if(GP1_Driver->GetRawButton(C_DRIVER_FAST_ADJ_BTN))
    {
        // Fine control mode; Scales driver input to smaller range for finer control
        Y_Demand = F_Scale(-1.0, 1.0, Throttle_Range_High, GP1_Driver->GetRawAxis(C_DRIVER_FWD_REV_AXIS)); //->GetLeftY());
        X_Demand = F_Scale(-1.0, 1.0, Throttle_Range_High, GP1_Driver->GetRawAxis(C_DRIVER_LFT_RHT_AXIS)); //->GetLeftX());
        Z_Demand = F_Scale(-1.0, 1.0, Throttle_Range_High, GP1_Driver->GetRawAxis(C_DRIVER_ROTATE_AXIS)); //->GetRightX());
    }
    else
    {
        Y_Demand = F_Scale(-1.0, 1.0, Throttle_Range_Normal, GP1_Driver->GetRawAxis(C_DRIVER_FWD_REV_AXIS)); //->GetLeftY());
        X_Demand = F_Scale(-1.0, 1.0, Throttle_Range_Normal, GP1_Driver->GetRawAxis(C_DRIVER_LFT_RHT_AXIS)); //->GetLeftX());
        Z_Demand = F_Scale(-1.0, 1.0, Throttle_Range_Normal, GP1_Driver->GetRawAxis(C_DRIVER_ROTATE_AXIS)); //->GetRightX());
        // Normal control mode
       // Y_Demand = F_Limit(Throttle_Range_Normal, GP1_Driver->GetRawAxis(C_DRIVER_FWD_REV_AXIS)); //->GetLeftY());
       // X_Demand = F_Limit(Throttle_Range_Normal, GP1_Driver->GetRawAxis(C_DRIVER_LFT_RHT_AXIS)); //->GetLeftX());
       // Z_Demand = F_Limit(Throttle_Range_Normal, GP1_Driver->GetRawAxis(C_DRIVER_ROTATE_AXIS)); //->GetRightX());
    }


    drivetrain_mode = GP1_Driver->GetRawButton(C_DRIVER_OCTO_SHIFT_BTN);


	X23._drivetrain.Drive(SC::F_Deadband(X_Demand, C_DRIVE_DEADBAND),
							SC::F_Deadband(Y_Demand, C_DRIVE_DEADBAND), 
							SC::F_Deadband(Z_Demand, C_DRIVE_DEADBAND), 
							true,
							drivetrain_mode);

	//X23._elevator.ControlDirectElevate(SC::F_Deadband(GP2_Operator->GetAxis(C_GD_J1_ELE_HIGHT), C_GD_DEADBAND));
	//X23._elevator.ControlDirectTilt(SC::F_Deadband(GP2_Operator->GetAxis(C_GD_J2_ELE_ANGLE), C_GD_DEADBAND));
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
