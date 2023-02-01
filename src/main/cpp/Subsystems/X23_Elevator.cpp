#include "Subsystems/X23_Elevator.h"
#include "units/angle.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "Constants.h"
#include "tuple"

using namespace SC;
using namespace frc;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace rev;

// Emc-Elevator Motor Control
X23_Elevator::X23_Elevator(std::tuple<int,int> Emc,int TiltMotor,SC::SC_Solenoid ChClawGripper, SC::SC_Solenoid ChClawTilt, int TiltHome,
    int ElevatorHome, int TiltMax)
{
// set elevate motors
    int sCh = C_DISABLED_CHANNEL;
    if(Emc != C_BLANK_IDS) 
    { 
        ElevateOne = new CANSparkMax(std::get<0>(Emc),rev::CANSparkMaxLowLevel::MotorType::kBrushless);
        ElevateOne->SetIdleMode (rev::CANSparkMax::IdleMode::kBrake);
         sCh = std::get<1>(Emc);

        if(sCh != C_DISABLED_CHANNEL) 
        { 
             ElevateTwo = new CANSparkMax (sCh, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
            ElevateTwo->SetIdleMode (rev::CANSparkMax::IdleMode::kBrake);
        }
		
        else { ElevateTwo = nullptr; }
    } 
    else
    {
        ElevateOne = nullptr;
        ElevateTwo = nullptr; 
    }
// Set tilt motor
    if(TiltMotor != C_DISABLED_CHANNEL) { TiltFalcon = new WPI_TalonFX (sCh); 
    TiltFalcon->SetNeutralMode(NeutralMode::Brake);
    TiltFalcon->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
	TiltFalcon->SetSelectedSensorPosition(0);
    }
	else { TiltFalcon = nullptr; }
//Set Digital Inputs
this->TiltHome = new DigitalInput(TiltHome);
this->TiltLimit = new DigitalInput(TiltMax);
this->ElevatorHome = new DigitalInput(ElevatorHome);

this->PincherSolenoid = new Solenoid(ChClawGripper.CtrlID, ChClawGripper.CtrlType, ChClawGripper.Channel);
	PincherSolenoid->Set(false);
this->TiltSolenoid = new Solenoid(ChClawTilt.CtrlID, ChClawTilt.CtrlType, ChClawTilt.Channel);
	TiltSolenoid->Set(false);
//set Debouncer
this->DebouncePincher = new Debouncer(C_Pincher_BTN_DBNC_TIME, frc::Debouncer::DebounceType::kRising);
this->DebounceTilt = new Debouncer(C_Pincher_BTN_DBNC_TIME, frc::Debouncer::DebounceType::kRising);
//set Trigger
this->rTrigPinch = new R_TRIG();
//set Bool
this->PincherSolenoidState = 0;
}


X23_Elevator::~X23_Elevator()
{
    if (Emc != nullptr) { delete Emc; Emc = nullptr; }
    if (rTrigPinch != nullptr) { delete rTrigPinch; rTrigPinch = nullptr; }

    if (ElevateOne != nullptr) { delete ElevateOne; ElevateOne = nullptr; }
    if (ElevateTwo != nullptr) { delete ElevateTwo; ElevateTwo = nullptr; }
    if (DebouncePincher != nullptr) { delete DebouncePincher; DebouncePincher = nullptr; }
    if (TiltLimit != nullptr) { delete TiltLimit; TiltLimit = nullptr; }

    if (TiltHome != nullptr) { delete TiltHome; TiltHome = nullptr; }
    if (ElevatorHome != nullptr) { delete ElevatorHome; ElevatorHome = nullptr; }
    if (TiltSolenoid != nullptr) { delete TiltSolenoid; TiltSolenoid = nullptr; }
    if (PincherSolenoid != nullptr) { delete PincherSolenoid; PincherSolenoid = nullptr; }
    if (TiltFalcon != nullptr) { delete TiltFalcon; TiltFalcon = nullptr; }
    if (ElevatorHome != nullptr) { delete ElevatorHome; ElevatorHome = nullptr; }
    if (TiltSolenoid != nullptr) { delete TiltSolenoid; TiltSolenoid = nullptr; }
    if (PincherSolenoid != nullptr) { delete PincherSolenoid; PincherSolenoid = nullptr; }
}

void X23_Elevator::Elevate(double TiltAngle, double ElevatorHeight)
{

}

void X23_Elevator::ToggleClaw(bool ClawToggleClose, bool ClawTiltDown)
{
<<<<<<< HEAD

    if((this->rTrigPinch != NULL)) 
=======
    /*
if((this-> != NULL))
>>>>>>> 38c31ee3095e035bcc9c3ca79aa2ed41d2f3061d
    {
        if((this->DebouncePincher != NULL) )
        {
            this->rTrigPinch->Check(this->DebouncePincher->Calculate(ClawToggleClose));
            
        }
        else
        {
// Debouncer's are not instantiated, read button change directly.
            this->rTrigPinch->Check(ClawToggleClose);
            
        }

        this->PincherSolenoidState  = (this->PincherSolenoidState && (!this->rTrigPinch->Q))     || (!this->PincherSolenoidState && this->rTrigPinch->Q);
    }
    else
    {
        if(this->DebouncePincher != NULL)
        {
            bool Claw;

<<<<<<< HEAD
            Claw = this->DebouncePincher->Calculate(ClawToggleClose);
         
            
=======
            s1e = this->_dbnc_s1e->Calculate(Stage1_Ext);
            s1c = this->_dbnc_s1c->Calculate(Stage1_Claw);

>>>>>>> 38c31ee3095e035bcc9c3ca79aa2ed41d2f3061d

            this->PincherSolenoidState = (this->PincherSolenoidState && !Claw) || (!this->PincherSolenoidState && Claw);
                        }
        else
        {
            this->PincherSolenoidState = (this->PincherSolenoidState && !ClawToggleClose) || (!this->PincherSolenoidState && ClawToggleClose);
                               }
    }

    if(this->PincherSolenoid != NULL) { this->PincherSolenoid->Set(this->PincherSolenoidState); }

// set debounce on tilt
    if(this->DebounceTilt != NULL)
    {
       if(this->TiltSolenoid != NULL) { this->TiltSolenoid->Set(this->DebounceTilt->Calculate(ClawTiltDown)); }  
            

}
    else
    {
    if(this->TiltSolenoid != NULL) { this->TiltSolenoid->Set(ClawTiltDown); }  



    }
}

void X23_Elevator::StopMotors()
{
    if(ElevateOne != nullptr) { ElevateOne->Set( 0.0); }
    if(TiltFalcon != nullptr) { TiltFalcon->Set(ControlMode::PercentOutput, 0.0); }
}
void X23_Elevator::ControlDirect(double RawElevate, double RawTiltFalcon)
{
    if(ElevateOne != nullptr) { ElevateOne->Set( F_Limit(-1.0, 1.0, RawElevate)); }

    if(TiltFalcon != nullptr) { TiltFalcon->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, RawTiltFalcon)); }
}