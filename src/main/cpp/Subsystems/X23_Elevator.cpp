#include "Subsystems/X23_Elevator.h"
#include "units/angle.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "Constants.h"
#include "tuple"
#include "cmath"
using namespace SC;
using namespace frc;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

// ElevateFalcon-Elevator Motor Control
X23_Elevator::X23_Elevator(int ElevateMotor,int TiltMotor,SC::SC_Solenoid ChClawGripper, SC::SC_Solenoid ChClawTilt,
SC::SC_Solenoid ChElevateBrake, int TiltHome, int ElevatorHome, int TiltMax)
{
// set elevate motors
if(ElevateMotor != C_DISABLED_CHANNEL) 
    {
    ElevateFalcon = new WPI_TalonFX (ElevateMotor); 
    ElevateFalcon->SetNeutralMode(NeutralMode::Brake);
    ElevateFalcon->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
	ElevateFalcon->SetSelectedSensorPosition(0);
    }
else 
    {
    ElevateFalcon = nullptr; 
    }
// Set tilt motor
    if(TiltMotor != C_DISABLED_CHANNEL) 
    { 
    TiltFalcon = new WPI_TalonFX (TiltMotor); 
    TiltFalcon->SetNeutralMode(NeutralMode::Brake);
    TiltFalcon->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
	TiltFalcon->SetSelectedSensorPosition(0);
    }
	else 
    {
    TiltFalcon = nullptr; 
    }
    //Set Digital Inputs
    this->TiltHome = new DigitalInput(TiltHome);
    this->TiltLimit = new DigitalInput(TiltMax);
    this->ElevatorHome = new DigitalInput(ElevatorHome);

    this->PincherSolenoid = new Solenoid(ChClawGripper.CtrlID, ChClawGripper.CtrlType, ChClawGripper.Channel);
    	PincherSolenoid->Set(false);
    
	this->TiltSolenoid = new Solenoid(ChClawTilt.CtrlID, ChClawTilt.CtrlType, ChClawTilt.Channel);
    	TiltSolenoid->Set(false);
    
	this->ElevateBrake = new Solenoid(ChElevateBrake.CtrlID, ChElevateBrake.CtrlType, ChElevateBrake.Channel);
    	ElevateBrake->Set(false);
    
	//set Debouncer
    this->DebouncePincher = new Debouncer(C_Pincher_BTN_DBNC_TIME, frc::Debouncer::DebounceType::kRising);
    this->DebounceTilt = new Debouncer(C_Pincher_BTN_DBNC_TIME, frc::Debouncer::DebounceType::kRising);
    
	//set Trigger
    this->rTrigPinch = new R_TRIG();
    //set Bool
    this->PincherSolenoidState = 0;
    //set variable values
    
E_CV = 0;
E_FooFighters = 0;
E_P = 0;
E_I_Min = 0;
E_I_Max = 100;
E_I = 0; 
E_Error_ZminusOne = 0;
E_D = 0;
T_CV = 0;
T_P = 0;
T_I_Min = 0;
T_I_Max = 100;
T_I = 0;
T_Error_ZminusOne = 0;
T_D = 0;
CalcHeight = 0;
CalcAngle = 0;
TiltAngleSP = 0;
ElevatorHeightSP = 0;
}


X23_Elevator::~X23_Elevator()
{
    if (rTrigPinch != nullptr) { delete rTrigPinch; rTrigPinch = nullptr; }

    if (ElevateFalcon != nullptr) { delete ElevateFalcon; ElevateFalcon = nullptr; }
    if (DebouncePincher != nullptr) { delete DebouncePincher; DebouncePincher = nullptr; }
    if (TiltLimit != nullptr) { delete TiltLimit; TiltLimit = nullptr; }

    if (TiltHome != nullptr) { delete TiltHome; TiltHome = nullptr; }
    if (ElevatorHome != nullptr) { delete ElevatorHome; ElevatorHome = nullptr; }
    if (TiltSolenoid != nullptr) { delete TiltSolenoid; TiltSolenoid = nullptr; }
    if (PincherSolenoid != nullptr) { delete PincherSolenoid; PincherSolenoid = nullptr; }
    if (TiltFalcon != nullptr) { delete TiltFalcon; TiltFalcon = nullptr; }
    if (DebounceTilt != nullptr) { delete DebounceTilt; DebounceTilt = nullptr; }
}

void X23_Elevator::Elevate()
{
    double Elevator_Error, Tilt_Error, ElevatePV, TiltPV;

    if(E_ntSP != nullptr) {ElevatorHeightSP = std::clamp(E_ntSP->GetDouble(0.0), 0.0, 68.5); } else { ElevatorHeightSP = 0.0; }
    if(E_ntKp != nullptr) {E_kpTune = E_ntKp->GetDouble(0.1); } else { E_kpTune = 0.0; }
    if(E_ntKi != nullptr) {E_kiTune = E_ntKi->GetDouble(0.1); } else { E_kiTune = 0.0; }
    if(E_ntKd != nullptr) {E_kdTune = E_ntKd->GetDouble(0.1); } else { E_kdTune = 0.0; }
    if(E_ntBias != nullptr) {E_BiasTune = E_ntBias->GetDouble(0.1); } else { E_BiasTune = 0.0; }
    
    
    if(T_ntSP != nullptr) {TiltAngleSP = std::clamp(T_ntSP->GetDouble(0.0), 0.0, 68.5); } else { TiltAngleSP = 0.0; }
    if(T_ntKp != nullptr) {T_kpTune = T_ntKp->GetDouble(0.1); } else { T_kpTune = 0.0; }
    if(T_ntKi != nullptr) {T_kiTune = T_ntKi->GetDouble(0.1); } else { T_kiTune = 0.0; }
    if(T_ntKd != nullptr) {T_kdTune = T_ntKd->GetDouble(0.1); } else { T_kdTune = 0.0; }
    if(T_ntBias != nullptr) {T_BiasTune = T_ntBias->GetDouble(0.1); } else { T_BiasTune = 0.0; }

    if(TiltAngleSP && ElevatorHeightSP == 0) {

    } else {

    }
    /*stops PID*/ 
    if((ElevatorHeightSP == 0) && (TiltAngleSP == 0 ) && atHome)
    {
        this->rTrigEHome->Check(ElevatorHome->Get());
        this->rTrigTHome->Check(TiltHome->Get());
        this->rTrigTLimit->Check(TiltLimit->Get());

        E_FooFighters = E_D = E_I = E_P = 0;
        ElevateFalcon->SetSelectedSensorPosition(0);
        
        T_D = T_I = T_P = 0;
        TiltFalcon->SetSelectedSensorPosition(0);   

    }
    //PID
    else if((ElevatorHeightSP != 0) && (TiltAngleSP != 0) && ElevatorHome != nullptr && TiltFalcon != nullptr && ElevateFalcon != nullptr && TiltLimit != nullptr && TiltHome != nullptr)
    {
        this->rTrigEHome->Check(ElevatorHome->Get());
        this->rTrigTHome->Check(TiltHome->Get());
        this->rTrigTLimit->Check(TiltLimit->Get());

        if (rTrigEHome->Q)
        {
            E_FooFighters = E_D = E_I = E_P = 0;
            ElevateFalcon->SetSelectedSensorPosition(0);
        }
        if (rTrigTHome->Q)
        {
            T_D = T_I = T_P = 0;   
        }
        else if (rTrigTLimit->Q)
        {
            T_D = T_I = T_P = TiltAngleSP-1;
        }
    //Define Locals
        CalcAngle = (F_XYCurve<double>(xArrayMotorPOS, yArrayAnglePOS, TiltPV, 10));
        ElevatePV = (ElevateFalcon->GetSelectedSensorPosition());
        TiltPV = (TiltFalcon->GetSelectedSensorVelocity(0));
    //second XY curve stuff for max height
        CalcHeight = fmin(F_XYCurve<double>(xArrayElevate, yArrayElevate, CalcAngle , 10 ), ElevatorHeightSP);
        Elevator_Error = CalcHeight - ElevatePV; 

    if (abs(Elevator_Error)<0.25) 
    {
        Elevator_Error = 0;
        Tilt_Error = 0; 
        E_CV = 0;
        ElevateBrake->Set(true);
    }
    //Elevater motor PID
    else
    {
        ElevateBrake->Set(false);

/*
        this->E_FooFighters = (F_XYCurve<double>(xArrayElevate, yArrayFooFighters, CalcAngle, 10));
        this->E_P = Elevator_Error * E_Kp;
        this->E_I = F_Limit(E_I_Max, E_I_Min, E_I+(E_Ki * Elevator_Error *E_dt));
        this->E_D = E_Kd * (Elevator_Error - E_Error_ZminusOne)/E_dt;
        this->E_CV = E_P + E_I + E_D + E_FooFighters;
*/
        this->E_FooFighters = (F_XYCurve<double>(xArrayElevate, yArrayFooFighters, CalcAngle, 10));
        this->E_P = Elevator_Error * E_kpTune;
        this->E_I = F_Limit(E_I_Max, E_I_Min, E_I+(E_kiTune * Elevator_Error *E_dt));
        this->E_D = E_kdTune * (Elevator_Error - E_Error_ZminusOne)/E_dt;
        this->E_CV = E_kdTune + E_kiTune + E_kpTune + E_FooFighters;

		// Tilt Motor PID
	    Tilt_Error = TiltAngleSP - CalcAngle;
       /* 
        this->T_P = Tilt_Error * T_Kp;
        this->T_I = F_Limit(T_I_Max, T_I_Min, T_I+(T_Ki * Tilt_Error *T_dt));
        this->T_D = T_Kd * (Tilt_Error - T_Error_ZminusOne)/T_dt;
        this->T_CV = T_P + T_I + T_D; 
        atHome = EHomeLS && THomeLS;
*/
        this->T_P = Tilt_Error * T_kpTune;
        this->T_I = F_Limit(T_I_Max, T_I_Min, T_I+(T_kiTune * Tilt_Error *T_dt));
        this->T_D = T_kdTune * (Tilt_Error - T_Error_ZminusOne)/T_dt; 
        E_CV = std::clamp<double>(E_P + E_I + E_D + E_FooFighters, -100, 100);
        T_CV = std::clamp<double>(T_P + T_I + T_D, -100, 100);
    }
    if(abs(Elevator_Error)>=0.25){

        
    }

        ElevateFalcon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,(E_CV/100.0));
        TiltFalcon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,(E_CV/100.0));

        atHome = EHomeLS && THomeLS;
        
        E_ntPV.Set(ElevatePV);
        T_ntPV.Set(TiltPV);
        T_ntAnglePV.Set(CalcAngle);
        E_ntCV.Set(E_CV);
        T_ntCV.Set(T_CV);
        E_ntP.Set(E_P);
        T_ntP.Set(T_P);
        E_ntI.Set(E_I);
        T_ntI.Set(T_I);
        E_ntD.Set(E_D);
        T_ntD.Set(T_D);
        E_ntErr.Set(Elevator_Error);
        T_ntErr.Set(Tilt_Error);
	}
}

frc2::CommandPtr X23_Elevator::ToggleClawOpen()
{
    return frc2::cmd::RunOnce([this]{
    if(this->PincherSolenoid != NULL) 
        { this->PincherSolenoid->Set(false); }});
}

frc2::CommandPtr X23_Elevator::ToggleClawShut()
{
    return frc2::cmd::RunOnce([this]{
    if(this->PincherSolenoid != NULL)
         { this->PincherSolenoid->Set(true); }});
}
// set debounce on tilt
frc2::CommandPtr X23_Elevator::ClawTilt(){
    return frc2::cmd::RunOnce([this]{
        if(this->TiltSolenoid != NULL) 
            {this->TiltSolenoid->Set(true); }});
}
frc2::CommandPtr X23_Elevator::StopTilt(){
    return frc2::cmd::RunOnce([this]{
        if(this->TiltSolenoid != NULL) 
            {this->TiltSolenoid->Set(false); }});
}
frc2::CommandPtr X23_Elevator::StopMotors()
{
    return frc2::cmd::RunOnce([this]{
    if(ElevateFalcon != nullptr) { ElevateFalcon->Set(0.0); }
    if(TiltFalcon != nullptr) { TiltFalcon->Set(ControlMode::PercentOutput, 0.0); }});
}

void X23_Elevator::ControlDirectElevate(double RawElevate)
{
    if(ElevateFalcon != nullptr) { ElevateFalcon->Set( F_Limit(-1.0, 1.0, RawElevate)); }
}
void X23_Elevator::ControlDirectTilt(double RawTiltFalcon)
{
    if(TiltFalcon != nullptr) { TiltFalcon->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, RawTiltFalcon)); }
}
frc2::CommandPtr X23_Elevator::HybridZone()
{
return frc2::cmd::RunOnce([this]{
ElevatorHeightSP = 14.5;
TiltAngleSP = 40;
});}
frc2::CommandPtr X23_Elevator::ConeOne()
{
return frc2::cmd::RunOnce([this]{
ElevatorHeightSP = 47.5;
TiltAngleSP = 34;
});}
frc2::CommandPtr X23_Elevator::ConeTwo()
{
return frc2::cmd::RunOnce([this]{
ElevatorHeightSP = 68.5;
TiltAngleSP = 40;
});}
frc2::CommandPtr X23_Elevator::CubeOne()
{
return frc2::cmd::RunOnce([this]{
ElevatorHeightSP = 41.5;
TiltAngleSP = 40;
});}
frc2::CommandPtr X23_Elevator::CubeTwo()
{
return frc2::cmd::RunOnce([this]{
ElevatorHeightSP = 68.5;
TiltAngleSP = 42;
});}
frc2::CommandPtr X23_Elevator::Substation()
{
return frc2::cmd::RunOnce([this]{
ElevatorHeightSP = 15;
TiltAngleSP = 38;
});}
frc2::CommandPtr X23_Elevator::HomePOS()
{
return frc2::cmd::RunOnce([this]{
ElevatorHeightSP = 0;
TiltAngleSP = 0;
});}
bool X23_Elevator::IsAtHome()
{
    return this->atHome;
}