#ifndef X23_Elevator_h
#define X23_Elevator_h

#include "frc/Solenoid.h"
#include "frc/DigitalInput.h"
#include "frc/filter/Debouncer.h"
#include "frc/filter/Debouncer.h"
 
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
<<<<<<< HEAD
#include "frc2/command/Commands.h"
=======
#include "networktables/NetworkTableInstance.h"
#include "networktables/DoubleTopic.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/GenericEntry.h>
>>>>>>> 7eea106614de941dc386d7c5c57c7373671eac85

#include "frc2/command/SubsystemBase.h"
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "FRC3484_Lib/utils/SC_Datatypes.h"

#include <frc/Joystick.h>

class X23_Elevator : public frc2::SubsystemBase
{
public:

    X23_Elevator(int ElevateMotor,int TiltMotor, SC::SC_Solenoid ChClawGripper, SC::SC_Solenoid ChClawTilt,
                SC::SC_Solenoid ChElevateBrake, int TiltHome, int ElevatorHome, int TiltMax);
     //Emc is Elevator motor Controller,Master and Slave//

    ~X23_Elevator();

    void Elevate();

    frc2::CommandPtr ToggleClawOpen();

    frc2::CommandPtr ToggleClawShut();

    frc2::CommandPtr ClawTilt();

    frc2::CommandPtr StopTilt();

    frc2::CommandPtr HybridZone();

    frc2::CommandPtr ConeOne();

    frc2::CommandPtr ConeTwo();

    frc2::CommandPtr CubeOne();

    frc2::CommandPtr CubeTwo();

    frc2::CommandPtr Substation();

    frc2::CommandPtr HomePOS();

    frc2::CommandPtr StopMotors();

    void ControlDirectElevate( double RawElevate);

    void ControlDirectTilt( double RawTiltFalcon);

    bool IsAtHome();

private:
    void _setOutputs();

    bool ClawToggleClose = 0;
    bool atHome = 0;
    bool EHomeLS = 0;
    bool THomeLS = 0;
// Declare Tilt 
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *TiltFalcon, *ElevateFalcon;  
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
double xArrayMotorPOS [10] {0, 1.19, 2.4, 3.63, 4.86, 6.08, 7.29, 8.47, 9.63, 10.08} ;//Actuator stroke position
double yArrayAnglePOS [10] {0, 14.875, 30, 45.375, 60.75, 76, 91.125, 105.875, 120.375, 126} ;//Tilt
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
double CalcHeight;//sp
double CalcAngle;
double TiltAngleSP;
double ElevatorHeightSP;//sp

};

/*===================*/
/* ELEVATOR COMMANDS */
/*===================*/

class Cmd_Elev_HybridZone : public frc2::CommandHelper<frc2::CommandBase, Cmd_Elev_HybridZone>
{ 
public:
    explicit Cmd_Elev_HybridZone(X23_Elevator &subsys) : _elevator(subsys) {} ;

    void Initialize() override;

    void Execute() override { if(_elevator.IsAtHome()) { _elevator.HybridZone(); } } ;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    X23_Elevator& _elevator;

		/*nt::DoubleSubcriber*/ nt::GenericEntry *ntSP, *ntKp, *ntKi, *ntkd, *ntBias, *ntMaxTravel;
		nt::DoublePublisher ntPV,ntCV, ntP, ntI, ntD, ntErr, ntManOut; 

		double P,I,D,CV;
		double err_last;

		double Kp = 0.1, Ki = 0.1, Kd = 0.1, dt = 0.01;
		double bias = 0;
		double SP;
		double output;

		double maxTravel;

		bool PID_En;

};


#endif // Elevator_H