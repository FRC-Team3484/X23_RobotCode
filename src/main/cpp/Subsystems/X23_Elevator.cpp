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
//this->DebouncePincher = new Debouncer()
}

X23_Elevator::~X23_Elevator()
{

}

void X23_Elevator::Elevate(double TiltAngle, double ElevatorHeight)
{

}

void X23_Elevator::ToggleClaw(bool ClawToggleClose, bool ClawTiltDown)
{
    /*
if((this-> != NULL)) 
    {
        if((this->_dbnc_s1e != NULL) && (this->_dbnc_s1e != NULL)) //&& (this->_dbnc_s2e != NULL) && (this->_dbnc_s2c != NULL))
        {
            this->rTrig_s1e->Check(this->_dbnc_s1e->Calculate(Stage1_Ext));
            this->rTrig_s1c->Check(this->_dbnc_s1c->Calculate(Stage1_Claw));
            // this->rTrig_s2e->Check(this->_dbnc_s2e->Calculate(Stage2_Ext));
            // this->rTrig_s2c->Check(this->_dbnc_s2c->Calculate(Stage2_Claw));
        }
        else
        {
            // Debouncer's are not instantiated, read button change directly.
            this->rTrig_s1e->Check(Stage1_Ext);
            this->rTrig_s1c->Check(Stage1_Claw);
            // this->rTrig_s2e->Check(Stage2_Ext);
            // this->rTrig_s2c->Check(Stage2_Claw);
        }

        this->stage1_ext_state  = (this->stage1_ext_state && (!this->rTrig_s1e->Q))     || (!this->stage1_ext_state && this->rTrig_s1e->Q);
        this->stage1_claw_state = (this->stage1_claw_state && (!this->rTrig_s1c->Q))    || (!this->stage1_claw_state && this->rTrig_s1c->Q);
        // this->stage2_ext_state  = (this->stage2_ext_state && (!this->rTrig_s2e->Q))     || (!this->stage2_ext_state && this->rTrig_s2e->Q);
        // this->stage2_claw_state = (this->stage2_claw_state && (!this->rTrig_s2c->Q))    || (!this->stage2_claw_state && this->rTrig_s2c->Q);
    }
    else
    {
        if((this->_dbnc_s1e != NULL) && (this->_dbnc_s1e != NULL)) // && (this->_dbnc_s2e != NULL) && (this->_dbnc_s2c != NULL))
        {
            bool s1e, s1c, s2e, s2c;

            s1e = this->_dbnc_s1e->Calculate(Stage1_Ext);
            s1c = this->_dbnc_s1c->Calculate(Stage1_Claw);
            

            this->stage1_ext_state = (this->stage1_ext_state && !s1e) || (!this->stage1_ext_state && s1e);
            this->stage1_claw_state = (this->stage1_claw_state && !s1c) || (!this->stage1_claw_state && s1c);
             }
        else
        {
            this->stage1_ext_state = (this->stage1_ext_state && !Stage1_Ext) || (!this->stage1_ext_state && Stage1_Ext);
            this->stage1_claw_state = (this->stage1_claw_state && !Stage1_Claw) || (!this->stage1_claw_state && Stage1_Claw);
                   }
    }

    if(this->_stage1_ext != NULL) { this->_stage1_ext->Set(this->stage1_ext_state); }
    if(this->_stage1_grab != NULL) { this->_stage1_grab->Set(this->stage1_claw_state); }

    */
}
