#include "Subsystems/X23_Intake.h"

#include "Constants.h" 
#include "Globals.h"

using namespace frc;
using namespace SC;
using namespace nt;
using namespace units::length;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


X23_Intake::X23_Intake(int IntakeID, int IntakeFeedMaster, int IntakeFeedSlave)
{

	Motor_Intake_Master = new VictorSPX(C_SPX_FEED_MASTER);
	Motor_Intake_Master->SetNeutralMode(NeutralMode::Coast);
	Motor_Intake_Master->SetInverted(true);
	Motor_Intake_Slave = new VictorSPX(C_SPX_FEED_SLAVE);
	Motor_Intake_Slave->SetNeutralMode(NeutralMode::Coast);
	Motor_Intake_Slave->SetInverted(true);

	this->_dbnc_rf_intake = new Debouncer(C_INTAKE_BTN_DBNC_TIME, Debouncer::kBoth); // Prevent accidental deployment or release of intake

	this->_dly_re_intake_on = new Debouncer(C_INTAKEMOTOR_DELAY_TIME, Debouncer::kRising); // Delay intake motors on until the intake is extended partially
	
}


X23_Intake::~X23_Intake()
{
	if(Motor_Intake_Master != NULL) { delete Motor_Intake_Master; }
	if(this->_dbnc_rf_intake != NULL) { delete this->_dbnc_rf_intake; }
	if(this->_dly_re_intake_on != NULL) { delete this->_dly_re_intake_on; }
}

void X23_Intake::Collect(bool Run, bool DirectionL, bool DirectionR)
{
	bool intakeOut, intakeForward, intakeReverse;
	double intakeSpeed;
	if(C_GD_INTAKE != false) {
			if(C_GD_DIRCET_L_INTAKE == false) {
			Motor_Intake_Master->Set(ControlMode::PercentOutput, 1.0);
			}
			else {
			Motor_Intake_Master->Set(ControlMode::PercentOutput, -1.0);
			}
			if(C_GD_DIRECT_R_INTAKE == false) {
			Motor_Intake_Slave->Set(ControlMode::PercentOutput, 1.0);
			}
			else {
			Motor_Intake_Slave->Set(ControlMode::PercentOutput, -1.0);
			}
	}
	// Set the motor speeds
	if(intakeForward) { intakeSpeed = C_INTAKE_DRIVE_SPEED; }
	else if(intakeReverse) { intakeSpeed = -C_INTAKE_DRIVE_SPEED; }
	else { intakeSpeed = 0.0;}

	// Apply outputs
	Motor_Intake_Master->Set(ControlMode::PercentOutput, intakeSpeed);
}