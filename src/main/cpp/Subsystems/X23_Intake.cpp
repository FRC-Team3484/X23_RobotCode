#include "Subsystems/X23_Intake.h"

#include "Constants.h" 
#include "Globals.h"
#include "frc2/command/RunCommand.h"

using namespace frc;
using namespace SC;
using namespace nt;
using namespace units::length;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


X23_Intake::X23_Intake(int IntakeLeft, int IntakeRight)
{
	Motor_Intake_Right = new VictorSPX(IntakeRight);
	Motor_Intake_Right->SetNeutralMode(NeutralMode::Brake);
	Motor_Intake_Right->SetInverted(false);

	Motor_Intake_Left = new VictorSPX(IntakeLeft);
	Motor_Intake_Left->SetNeutralMode(NeutralMode::Brake);
	Motor_Intake_Left->SetInverted(false);

	this->_dbnc_rf_intake = new Debouncer(C_INTAKE_BTN_DBNC_TIME, Debouncer::kBoth); // Prevent accidental deployment or release of intake

	this->_dly_re_intake_on = new Debouncer(C_INTAKEMOTOR_DELAY_TIME, Debouncer::kRising); // Delay intake motors on until the intake is extended partially
	
	this->SetDefaultCommand(frc2::RunCommand([&]() { StopIntake(); }, {this}));

}


X23_Intake::~X23_Intake()
{
	if(Motor_Intake_Right != NULL) { delete Motor_Intake_Right; }
	if(Motor_Intake_Left != NULL) { delete Motor_Intake_Left;  }
	if(this->_dbnc_rf_intake != NULL) { delete this->_dbnc_rf_intake; }
	if(this->_dly_re_intake_on != NULL) { delete this->_dly_re_intake_on; }
}

void X23_Intake::Collect(bool Run, bool ButtonA, bool ButtonB, bool ButtonC, bool ButtonD)
{
	if(Motor_Intake_Left != nullptr && Motor_Intake_Right != nullptr)
	{
		if(Run != false) 
		{
			if(ButtonA != false) 
			{
				//Suck In Cubes
				Motor_Intake_Right->Set(ControlMode::PercentOutput, 1.0);
				Motor_Intake_Left->Set(ControlMode::PercentOutput, -1.0);
			}
			else if(ButtonB != false)
			{
				//Knock Cone One Way
				Motor_Intake_Right->Set(ControlMode::PercentOutput, -1.0);
				Motor_Intake_Left->Set(ControlMode::PercentOutput, -1.0);
			}
			else if(ButtonC != false)
			{
				//Knock Cone Other Way
				Motor_Intake_Right->Set(ControlMode::PercentOutput, 1.0);
				Motor_Intake_Left->Set(ControlMode::PercentOutput, 1.0);
			}
			else if(ButtonD != false)
			{
				//Sphit Out Cubes
				Motor_Intake_Right->Set(ControlMode::PercentOutput, -1.0);
				Motor_Intake_Left->Set(ControlMode::PercentOutput, 1.0);
			}
			else
			{
				Motor_Intake_Right->Set(ControlMode::PercentOutput, 0.0);
				Motor_Intake_Left->Set(ControlMode::PercentOutput, 0.0);	
			}
		}
		else
		{
			Motor_Intake_Right->Set(ControlMode::PercentOutput, 0.0);
			Motor_Intake_Left->Set(ControlMode::PercentOutput, 0.0);	
		}
	}
}

/* Commands */
void X23_Intake::Collect_ConeLeft()
{

}

void X23_Intake::StopIntake()
{

}