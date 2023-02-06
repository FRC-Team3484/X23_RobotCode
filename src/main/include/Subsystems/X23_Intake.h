#ifndef SC_Intake_h
#define SC_Intake_h

#include "FRC3484_Lib/components/SC_Limelight.h"
#include "FRC3484_Lib/components/SC_ColorSensor.h"
#include "FRC3484_Lib/components/SC_PID.h"

#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "FRC3484_Lib/utils/SC_Shuffleboard.h"

#include "ctre/phoenix/motorcontrol/can/TalonFX.h"
#include "ctre/phoenix/motorcontrol/can/VictorSPX.h"

#include "frc/Solenoid.h"
#include "frc/DigitalInput.h"
#include "frc/filter/Debouncer.h"

#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableValue.h"


class X23_Intake
{

public:
	X23_Intake(int IntakeID, int IntakeMaster, int IntakeSlave);
	void Collect(bool Intake, bool DirectionL, bool DirecttionR);

private:

	void _initDashboard();
	void _updateDashboard();
	void _getColorDist();

	ctre::phoenix::motorcontrol::can::VictorSPX *Motor_Intake_Master;
	ctre::phoenix::motorcontrol::can::VictorSPX *Motor_Intake_Slave;

	nt::NetworkTableInstance _nt_inst;
	std::shared_ptr<nt::NetworkTable> _nt_table;

	// Debouncers
	frc::Debouncer *_dbnc_rf_intake;        // Debounce driver's `Intake` command on both the rising edge (R) and falling edge (F)

	// Delay timers
	/* Debouncers can double as a delay timer for triggering an action after a specified time after a signal changes */
	frc::Debouncer *_dly_re_intake_on;  // Delay intake motors on after the rising edge (RE) of the driver's `Intake` command
};




#endif //SC_Intake_h
