#ifndef SC_Intake_h
#define SC_Intake_h

#include "FRC3484_Lib/components/SC_Limelight.h"
#include "FRC3484_Lib/components/SC_ColorSensor.h"
#include "FRC3484_Lib/components/SC_PID.h"

#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "FRC3484_Lib/utils/SC_Functions.h"

#include "ctre/phoenix/motorcontrol/can/TalonFX.h"
#include "ctre/phoenix/motorcontrol/can/VictorSPX.h"

#include "frc/Solenoid.h"
#include "frc/DigitalInput.h"
#include "frc/filter/Debouncer.h"



class X23_Intake
{

public:
	X23_Intake(int IntakeLeft, int IntakeRight);
	void Collect(bool Run, bool ButtonA, bool ButtonB, bool ButtonC, bool ButtonD);
~X23_Intake();

private:
	void _getColorDist();

	ctre::phoenix::motorcontrol::can::VictorSPX *Motor_Intake_Right;
	ctre::phoenix::motorcontrol::can::VictorSPX *Motor_Intake_Left;

	// Debouncers
	frc::Debouncer *_dbnc_rf_intake;        // Debounce driver's `Intake` command on both the rising edge (R) and falling edge (F)

	// Delay timers
	/* Debouncers can double as a delay timer for triggering an action after a specified time after a signal changes */
	frc::Debouncer *_dly_re_intake_on;  // Delay intake motors on after the rising edge (RE) of the driver's `Intake` command
};




#endif //SC_Intake_h
