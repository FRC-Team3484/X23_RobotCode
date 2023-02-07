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
    rev::SparkMaxRelativeEncoder ElevateEncoder{};
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
//PID Elevator Loop
double E_CV;
double E_P;
double E_I_Min;
double E_I_Max;
double E_I 0;
double xArrayElevate [9] = [ 5, 10, 15, 20, 25, 30, 35, 40, 45];
double yArrayElevate [9];
double E_Error_ZminusOne;
double E_D;
double T_CV;
double T_P;
double T_I_Min;
double T_I_Max;
double T_I;
double T_Error_ZminusOne;
double T_D;
};

#endif // Elevator_H