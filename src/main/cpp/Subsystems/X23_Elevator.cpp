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
	if(ElevateMotor != C_DISABLED_CHANNEL) { ElevateFalcon = new WPI_TalonFX (ElevateMotor); 
        TiltFalcon->SetNeutralMode(NeutralMode::Brake);
        TiltFalcon->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
	    TiltFalcon->SetSelectedSensorPosition(0);
    }
	else { TiltFalcon = nullptr; }
	
	// Set tilt motor
    if(TiltMotor != C_DISABLED_CHANNEL) { TiltFalcon = new WPI_TalonFX (TiltMotor); 
        TiltFalcon->SetNeutralMode(NeutralMode::Brake);
        TiltFalcon->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
	    TiltFalcon->SetSelectedSensorPosition(0);
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
    this->PincherSolenoidState = false;
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
	if(ElevatorHome != nullptr && TiltFalcon != nullptr && ElevateFalcon != nullptr && TiltLimit != nullptr && TiltHome != nullptr)
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
			T_D = T_I = T_P = 0;
		else if (rTrigTLimit->Q)
			T_D = T_I = T_P = 0;

//Define Locals
        double Elevator_Error, Tilt_Error;
        CalcAngle = (F_XYCurve<double>(xArrayMotorPOS, yArrayAnglePOS,TiltFalcon->GetSelectedSensorVelocity(0), 10));

//second XY curve stuff for max height
        CalcHeight = fmin(F_XYCurve<double>(xArrayElevate, yArrayElevate, CalcAngle , 10 ), ElevatorHeight);
        Elevator_Error = CalcHeight - ElevateFalcon->GetSelectedSensorPosition(); 

        this->E_FooFighters = (F_XYCurve<double>(xArrayElevate, yArrayFooFighters, CalcAngle, 10));
        this->E_P = Elevator_Error * E_Kp;
        this->E_I = F_Limit(E_I_Max, E_I_Min, E_I+(E_Ki * Elevator_Error *E_dt));
        this->E_D = E_Kd * (Elevator_Error - E_Error_ZminusOne)/E_dt;
        this->E_CV = E_P + E_I + E_D + E_FooFighters;

		// Tilt Motor PID
   		Tilt_Error = TiltAngle - CalcAngle;
        
        this->T_P = Tilt_Error * T_Kp;
        this->T_I = F_Limit(T_I_Max, T_I_Min, T_I+(T_Ki * Tilt_Error *T_dt));
        this->T_D = T_Kd * (Tilt_Error - T_Error_ZminusOne)/T_dt;
        this->T_CV = T_P + T_I + T_D; 
	}
}

void X23_Elevator::ToggleClaw()
{
    if((this->rTrigPinch != NULL)) 
    {
        if((this->DebouncePincher != NULL) )
            this->rTrigPinch->Check(this->DebouncePincher->Calculate(ClawToggleClose));
        else // Debouncer's are not instantiated, read button change directly.
            this->rTrigPinch->Check(ClawToggleClose);
    	
		this->PincherSolenoidState  = (this->PincherSolenoidState && (!this->rTrigPinch->Q)) || (!this->PincherSolenoidState && this->rTrigPinch->Q);
    }
    else
    {
        if(this->DebouncePincher != NULL)
        {
            bool Claw;
            Claw = this->DebouncePincher->Calculate(ClawToggleClose);

            this->PincherSolenoidState = (this->PincherSolenoidState && !Claw) || (!this->PincherSolenoidState && Claw);
        }
        else
            this->PincherSolenoidState = (this->PincherSolenoidState && !ClawToggleClose) || (!this->PincherSolenoidState && ClawToggleClose);
    }

    if(this->PincherSolenoid != NULL) { this->PincherSolenoid->Set(this->PincherSolenoidState); }
}
// set debounce on tilt
void X23_Elevator::ToggleTilt(bool ClawTiltDown){
    if(this->DebounceTilt != NULL)
       if(this->TiltSolenoid != NULL) { this->TiltSolenoid->Set(this->DebounceTilt->Calculate(ClawTiltDown)); } 
    else
        if(this->TiltSolenoid != NULL) { this->TiltSolenoid->Set(ClawTiltDown); } 
}

void X23_Elevator::StopMotors()
{
    if(ElevateFalcon != nullptr) { ElevateFalcon->Set(0.0); }
    if(TiltFalcon != nullptr) { TiltFalcon->Set(ControlMode::PercentOutput, 0.0); }
}

void X23_Elevator::ControlDirect(double RawElevate, double RawTiltFalcon)
{
    if(ElevateFalcon != nullptr) { ElevateFalcon->Set( F_Limit(-1.0, 1.0, RawElevate)); }
    if(TiltFalcon != nullptr) { TiltFalcon->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, RawTiltFalcon)); }
}