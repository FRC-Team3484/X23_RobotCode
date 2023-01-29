#include "Subsystems/X23_Elevator.h"
#include "units/angle.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "Constants.h"
#include "tuple"
#include "Subsystems/X23_Elevator.h"

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
//this->DebouncePincher = new Debouncer()
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
  
}
