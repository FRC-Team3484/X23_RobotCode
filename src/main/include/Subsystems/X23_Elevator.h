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
    X23_Elevator(std::tuple<int,int> ElevateMotor,int TiltMotor,SC::SC_Solenoid ChClawGripper, SC::SC_Solenoid ChClawTilt,
SC::SC_Solenoid ChElevateBrake, int TiltHome, int ElevatorHome, int TiltMax);
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
    //rev::SparkMaxRelativeEncoder ElevateEncoder;
// Declare Pincher Solenoids
    frc::Solenoid *PincherSolenoid;
// Declare Pincher Tilt Solenoids
    frc::Solenoid *TiltSolenoid, *ElevateBrake;
// Declare Limit Switches
    frc::DigitalInput *TiltLimit, *TiltHome, *ElevatorHome;
//Debouncers
	bool PincherSolenoidState;

    SC::R_TRIG *rTrigPinch, *rTrigEHome, *rTrigTHome, *rTrigTLimit;
    frc::Debouncer *DebouncePincher, *DebounceTilt;
//PID Elevator Loop
double E_CV;
double E_FooFighters;
double E_P;
double E_I_Min;
double E_I_Max;
double E_I = 0; 
double xArrayMotorPOS [10] ;//Actuator stroke position
double yArrayAnglePOS [10] ;//Tilt
double yArrayFooFighters [10] {10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
double xArrayElevate [10] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 42};
double yArrayElevate [10] = {56.5, 56.5, 57.5, 59.5, 61.5, 64.5, 67.5, 68.5, 68.5, 68.5};
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