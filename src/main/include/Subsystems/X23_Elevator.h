#ifndef X23_Elevator_h
#define X23_Elevator_h

#include "frc/Solenoid.h"
#include "frc/DigitalInput.h"
#include "frc/filter/Debouncer.h"
 
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include "networktables/NetworkTableInstance.h"
#include <networktables/GenericEntry.h>
#include "networktables/DoubleTopic.h"
#include "networktables/BooleanTopic.h"

#include "frc2/command/SubsystemBase.h"
#include <frc2/command/Command.h>
#include "frc2/command/Commands.h"
#include <frc2/command/CommandHelper.h>

#include "FRC3484_Lib/utils/SC_Datatypes.h"

#include "Constants.h"



class X23_Elevator : public frc2::SubsystemBase
{
public:

    X23_Elevator(int ElevateMotor,int TiltMotor, SC::SC_Solenoid ChClawGripper, SC::SC_Solenoid ChClawTilt,
                SC::SC_Solenoid ChElevateBrake, int TiltHomeCh, int ElevatorHomeCh, int TiltMaxCh);
     //Emc is Elevator motor Controller,Master and Slave//

    ~X23_Elevator();

    void Elevate();

    bool pidDisabled();

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

    void InitNetworkTables();

private:
    void _setOutputs();
	bool _arePointersValid();

	double GetElevatorHeightLimit(double tiltAngle);
	double GetTiltAngle(double TiltInches);

    bool ClawToggleClose = 0;
    bool atHome = 0;
    bool EHomeLS = 0;
    bool THomeLS = 0;
    bool TMaxTravelLS = false;

    bool E_PID_isDisabled = 0;
    bool T_PID_isDisabled = 0;
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
	bool TiltClawSolenoidState;

    SC::R_TRIG *rTrigPinch, *rTrigEHome, *rTrigTHome, *rTrigTLimit;
    frc::Debouncer *DebouncePincher, *DebounceTilt;

	//PID Elevator Loop
	double E_CV;
	double E_FooFighters;
	double E_P;
	double E_I_Min;
	double E_I_Max;
	double E_I = 0; 
	double ArrayActuatorPOS [10] {0, 1.19, 2.4, 3.63, 4.86, 6.08, 7.29, 8.47, 9.63, 10.08} ;//Actuator stroke position
	double yArrayAnglePOS [10] {0, 14.875, 30, 45.375, 60.75, 76, 91.125, 105.875, 120.375, 126} ;//Tilt
	double yArrayFooFighters [10] {10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
	double AngleArray [10] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 42};//angle
	//double ElevateHeightArray [10] = {56.5, 56.5, 57.5, 59.5, 61.5, 64.5, 67.5, 68.5, 68.5, 68.5};
	//double ElevateHeightArray [10] = {15.5, 15.5, 16.5, 18.5, 20.5, 23.5, 24.5, 27.5, 27.5, 27.5};
	//double ElevateHeightArray [10] = {36.0, 36.0, 37.5, 39.5, 41.5, 44.5, 47.5, 48.5, 48.5, 48.5};
	double ElevateHeightArray [10] = {36.0, 36.0, 37.5, 39.5, 61.5, 64.5, 67.5, 68.5, 68.5, 68.5};
	double E_Error_ZminusOne;
	double E_D;
	double T_CV;
	double T_P;
	double T_I_Min;
	double T_I_Max;
	double T_I;
	double T_Error_ZminusOne;
	double T_D;
	double CalcHeight;
	double CalcAngle;
	double TiltAngleSP;
	double ElevatorHeightSP;//sp

#ifdef C_BUILD_OPT_ELEV_TUNING
	double E_kpTune;
	double E_kiTune;
	double E_kdTune;
	double E_BiasTune;
	double E_spTune;
	double T_kpTune;
	double T_kiTune;
	double T_kdTune;
	double T_BiasTune;
	double T_spTune;

	nt::GenericEntry *E_ntSP, *E_ntKp, *E_ntKi, *E_ntKd, *E_ntBias;
	nt::GenericEntry *T_ntSP, *T_ntKp, *T_ntKi, *T_ntKd, *T_ntBias;
#endif
	// Outputs

	nt::GenericEntry *E_ntPV, *E_ntCV, *E_ntP, *E_ntI, *E_ntD, *E_ntErr;
	nt::GenericEntry *T_ntPV, *T_ntCV, *T_ntP, *T_ntI, *T_ntD, *T_ntErr, *T_ntAnglePV;
	nt::GenericEntry *E_ntAtHome, *T_ntAtHome, *E_ntPIDDisabled, *T_ntPIDDisabled, *T_ntAtMax;

	//nt::DoublePublisher E_ntPV,E_ntCV, E_ntP, E_ntI, E_ntD, E_ntErr;
	//nt::DoublePublisher T_ntPV, T_ntCV, T_ntP, T_ntI, T_ntD, T_ntErr, T_ntAnglePV;
	//nt::BooleanPublisher E_ntAtHome, T_ntAtHome, E_ntPIDDisabled, T_ntPIDDisabled;
};

/*===================*/
/* ELEVATOR COMMANDS */
/*===================*/



#endif // Elevator_H
