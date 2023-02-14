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

#include "frc2/command/SubsystemBase.h"


class X23_Intake : public frc2::SubsystemBase
{
public:
	X23_Intake(int IntakeLeft, int IntakeRight);
	~X23_Intake();

	void Collect(bool Run, bool ButtonA, bool ButtonB, bool ButtonC, bool ButtonD);

	/*-------------------*/
	/* Command functions */
	/*-------------------*/
	/*
		Intake Orientations
			All cone orientations reference the the direction the cone
			is pointing while laying on its side starting from the base 
			and moving towards the top of the cone.

		Left orientation:
			Left mandible:	CW
			Right mandible:	CW

		Right orientation:
			Left mandible:	CCW
			Right mandible:	CCW

		Center orientation:
			Left mandible: CW
			Right mandible: CCW

		Cube:
			Left mandible:	CW
			Right mandible:	CCW

		Eject
			Left mandible:	CCW
			Right mandible:	CW
	*/

	/* Run the intake for left-oriented cones */
	void Collect_ConeLeft();

	/* Run the intake for right-oriented cones */
	void Collect_ConeRight();

	/*Run the intake for center-oriented cones*/
	void Collect_Cube_Or_ConeCenter();

	/*Run the ejection procedure*/
	void Collect_Eject();

	/* Stop the intake motors */
	void StopIntake();

private:

	ctre::phoenix::motorcontrol::can::VictorSPX *Motor_Intake_Right;
	ctre::phoenix::motorcontrol::can::VictorSPX *Motor_Intake_Left;

	// Debouncers
	frc::Debouncer *_dbnc_rf_intake;        // Debounce driver's `Intake` command on both the rising edge (R) and falling edge (F)

	// Delay timers
	/* Debouncers can double as a delay timer for triggering an action after a specified time after a signal changes */
	frc::Debouncer *_dly_re_intake_on;  // Delay intake motors on after the rising edge (RE) of the driver's `Intake` command
};




#endif //SC_Intake_h
