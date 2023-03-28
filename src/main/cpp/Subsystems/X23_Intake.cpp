#include "Subsystems/X23_Intake.h"

#include "Constants.h" 
#include "Globals.h"
#include "frc2/command/RunCommand.h"

using namespace frc;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


X23_Intake::X23_Intake(int Intake)
{
	Motor_Intake = new VictorSPX(Intake);
	Motor_Intake->SetNeutralMode(NeutralMode::Brake);
	Motor_Intake->SetInverted(false);

	this->_dbnc_rf_intake = new Debouncer(C_INTAKE_BTN_DBNC_TIME, Debouncer::kBoth); // Prevent accidental deployment or release of intake

	this->_dly_re_intake_on = new Debouncer(C_INTAKEMOTOR_DELAY_TIME, Debouncer::kRising); // Delay intake motors on until the intake is extended partially
	
	this->SetDefaultCommand(frc2::RunCommand([&]() { StopIntake(); }, {this}));

}


X23_Intake::~X23_Intake()
{
	if(Motor_Intake != NULL) { delete Motor_Intake; }
	if(this->_dbnc_rf_intake != NULL) { delete this->_dbnc_rf_intake; }
	if(this->_dly_re_intake_on != NULL) { delete this->_dly_re_intake_on; }
}

void X23_Intake::Collect(bool ButtonA, bool ButtonB, bool ButtonC, bool ButtonD)
{
	if(Motor_Intake)
	{
		if(ButtonA) 
		{
			//Suck In Cubes $
			Motor_Intake->Set(ControlMode::PercentOutput, C_INTAKE_CW_SPEED);
		}
		else if(ButtonD)
		{
			//Sphit Out Cubes
			Motor_Intake->Set(ControlMode::PercentOutput, C_INTAKE_CCW_SPEED);
		}
		else
		{
			Motor_Intake->Set(ControlMode::PercentOutput, C_INTAKE_CW_SPEED);
		}
	}
	//else
	//{
	//	Motor_Intake_Right->Set(ControlMode::PercentOutput, 0.0);
	//	Motor_Intake_Left->Set(ControlMode::PercentOutput, 0.0);	
	//}
}

/* Commands */
frc2::CommandPtr X23_Intake::Collect()
{
	return frc2::cmd::RunOnce([this]{
	//Suck The Cubical Object!
	Motor_Intake->Set(ControlMode::PercentOutput, C_INTAKE_CCW_SPEED);
	});
}
frc2::CommandPtr X23_Intake::Collect_Eject()
{
	return frc2::cmd::RunOnce([this]{
	//Eject Cubes
	Motor_Intake->Set(ControlMode::PercentOutput, C_INTAKE_CW_SPEED);});
}

frc2::CommandPtr X23_Intake::StopIntake()
{
	return frc2::cmd::RunOnce([this]{
	//Stop
	Motor_Intake->Set(ControlMode::PercentOutput, 0.0);});
}
