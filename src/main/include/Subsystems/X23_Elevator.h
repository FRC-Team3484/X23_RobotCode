#ifndef X23_Elevator_h
#define X23_Elevator_h

#include "frc/Solenoid.h"
#include "FRC3484_Lib/components/SC_PID.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "frc/DigitalInput.h"
#include "frc/filter/Debouncer.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "rev/CANSparkMax.h"
#include "frc/filter/Debouncer.h"

class X23_Elevator
{
public:
    X23_Elevator(std::tuple<int,int> Emc,int TiltMotor, 
    SC::SC_Solenoid ChClawGripper, SC::SC_Solenoid ChClawTilt, SC::SC_Solenoid ChElevateBrake,
    int TiltHome, int ElevatorHome, int TiltMax);
     //Emc is Elevator motor Controller,Master and Slave//

 ~X23_Elevator();

    void Elevate(double TiltAngle, double ElevatorHeight);

    void ToggleClaw(bool ClawToggleClose, bool ClawTiltDown);

    void StopMotors();

    void ControlDirect(double RawElevate, double RawTiltFalcon);

private:
    void _setOutputs();

// Declare Tilt 
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *TiltFalcon;  
// Declare Neo
    rev::CANSparkMax *ElevateOne, *ElevateTwo;
// Declare Pincher Solenoids
    frc::Solenoid *PincherSolenoid;
// Declare Pincher Tilt Solenoids
    frc::Solenoid *TiltSolenoid, *ElevateBrake;
// Declare Limit Switches
    frc::DigitalInput *TiltLimit, *TiltHome, *ElevatorHome;

//Debouncers
	bool PincherSolenoidState;

    SC::R_TRIG *rTrigPinch;
    frc::Debouncer *DebouncePincher, *DebounceTilt;
};

#endif // Elevator_H