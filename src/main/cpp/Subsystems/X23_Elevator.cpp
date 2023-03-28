#include "Subsystems/X23_Elevator.h"
#include "units/angle.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "Constants.h"
#include "tuple"
#include "cmath"
#include "frc/RobotState.h"


using namespace SC;
using namespace frc;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

// ElevateFalcon-Elevator Motor Control
X23_Elevator::X23_Elevator(int ElevateMotor, int TiltMotor, SC::SC_Solenoid ChClawGripper, SC::SC_Solenoid ChClawTilt,
							SC::SC_Solenoid ChElevateBrake, int TiltHomeCh, int ElevatorHomeCh, int TiltMaxCh)
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
        TiltFalcon->SetInverted(true);
    }
	else 
    {
        TiltFalcon = nullptr; 
    }

    //Set Digital Inputs
    this->TiltHome = new DigitalInput(TiltHomeCh);
    this->TiltLimit = new DigitalInput(TiltMaxCh);
    this->ElevatorHome = new DigitalInput(ElevatorHomeCh);

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
	this->rTrigEHome = new R_TRIG();
	this->rTrigTHome = new R_TRIG();
	this->rTrigTLimit = new R_TRIG();

    //set Bool
    this->PincherSolenoidState = 0;

    //set variable values

    E_CV = 0;
    E_FooFighters = 0;
    E_P = 0;
    E_I_Min = -100;
    E_I_Max = 100;
    E_I = 0; 
    E_Error_ZminusOne = 0;
    E_D = 0;

    T_CV = 0;
    T_P = 0;
    T_I_Min = -100;
    T_I_Max = 100;
    T_I = 0;
    T_Error_ZminusOne = 0;
    T_D = 0;

    CalcHeight = 0;
    CalcAngle = 0;
    TiltAngleSP = 0;
    ElevatorHeightSP = 0;

    E_PID_isDisabled = true;
    T_PID_isDisabled = true;
    EHomeLS = false;
    THomeLS = false;
    TMaxTravelLS = false;
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

bool X23_Elevator::pidDisabled()
{
    return E_PID_isDisabled && T_PID_isDisabled;
}    

void X23_Elevator::Periodic()
{
	if(!frc::RobotState::IsDisabled())
	{
		if (_arePointersValid())
		{
			double Elevator_Error = 0, Tilt_Error = 0, ElevatePV = 0, TiltPV = 0;

	#ifdef C_BUILD_OPT_ELEV_TUNING
	//E_PID_isDisabled = false;
	//T_PID_isDisabled = false;
			if(E_ntSP != nullptr)   {ElevatorHeightSP = std::clamp(E_ntSP->GetDouble(0.0), 0.0, 68.5); } else { ElevatorHeightSP = 0.0; }
			if(E_ntKp != nullptr)   {E_kpTune = E_ntKp->GetDouble(0.1); } else { E_kpTune = 0.0; }
			if(E_ntKi != nullptr)   {E_kiTune = E_ntKi->GetDouble(0.1); } else { E_kiTune = 0.0; }
			if(E_ntKd != nullptr)   {E_kdTune = E_ntKd->GetDouble(0.1); } else { E_kdTune = 0.0; }
			if(E_ntBias != nullptr) {E_BiasTune = E_ntBias->GetDouble(0.1); } else { E_BiasTune = 0.0; }
			
		
			if(T_ntSP != nullptr) {TiltAngleSP = std::clamp(T_ntSP->GetDouble(0.0), 0.0, 45.0); } else { TiltAngleSP = 0.0; }
			if(T_ntKp != nullptr) {T_kpTune = T_ntKp->GetDouble(0.1); } else { T_kpTune = 0.0; }
			if(T_ntKi != nullptr) {T_kiTune = T_ntKi->GetDouble(0.1); } else { T_kiTune = 0.0; }
			if(T_ntKd != nullptr) {T_kdTune = T_ntKd->GetDouble(0.1); } else { T_kdTune = 0.0; }
			if(T_ntBias != nullptr) {T_BiasTune = T_ntBias->GetDouble(0.1); } else { T_BiasTune = 0.0; }
	#endif
			EHomeLS = ElevatorHome->Get();
			THomeLS = TiltHome->Get();
			ElevateBrake->Set(E_PID_isDisabled);
			atHome = EHomeLS && THomeLS;

			ElevatePV = (ElevateFalcon->GetSelectedSensorPosition(0) * C_ELE_SCALE_FACTOR_POSN + 6.0);
			TiltPV = (TiltFalcon->GetSelectedSensorPosition(0) * C_TILT_SCALE_FACTOR_POSN);

			/* stops PID */ 

					if (EHomeLS) //&& !(ElevatorHeightSP > 0))
					{
						ElevateFalcon->SetSelectedSensorPosition(0);
						//E_PID_isDisabled = true;
					}
					
					if (THomeLS && !(TiltAngleSP > 0))
					{ 
						TiltFalcon->SetSelectedSensorPosition(0);
						//T_PID_isDisabled = true;
					}
					else if (TiltLimit->Get())
					{
						TiltAngleSP = TiltAngleSP-1;
					}
			
					//Define Locals
					CalcAngle = (F_XYCurve<double>(ArrayActuatorPOS , AngleArray, TiltPV, 10));
					
					//second XY curve stuff for max height
					CalcHeight = fmin(F_XYCurve<double>(AngleArray, ElevateHeightArray, CalcAngle , 10 ), ElevatorHeightSP);
					Tilt_Error = TiltAngleSP - CalcAngle;
					Elevator_Error = CalcHeight - ElevatePV; 
					{
						
						//ElevateBrake->Set(false);

						T_PID_isDisabled = ((abs(Tilt_Error) < 1.0) && (TiltAngleSP != 0)) || (THomeLS && (TiltAngleSP == 0));
						E_PID_isDisabled = ((abs(Elevator_Error) < 1.0) && (ElevatorHeightSP != 0)) || (EHomeLS && (ElevatorHeightSP == 0));
//											|| (E_PID_isDisabled && (ElevatorHeightSP == 0));

	#ifndef C_BUILD_OPT_ELEV_TUNING					
						if(!E_PID_isDisabled)
						{
							// Elevator Motor PID
							this->E_FooFighters = (F_XYCurve<double>(ElevateHeightArray, yArrayFooFighters, CalcAngle, 10));
							this->E_P = Elevator_Error * E_kp;
							this->E_I = F_Limit(E_I_Max, E_I_Min, E_I+(E_ki * Elevator_Error *E_dt));
							this->E_D = E_kd * (Elevator_Error - E_Error_ZminusOne)/E_dt;
							E_Error_ZminusOne = Elevator_Error;
						}
						else
						{
							E_Error_ZminusOne = 0;
							E_FooFighters = E_D = E_I = E_P = 0;
						}


						if(!T_PID_isDisabled )
						{
							// Tilt Motor PID
							this->T_FooFighters = std::copysign(CalcAngle, TiltAngleSP) * T_Bias;
							this->T_P = Tilt_Error * T_kp;
							this->T_I = F_Limit(T_I_Max, T_I_Min, T_I+(T_ki * Tilt_Error *T_dt));
							this->T_D = T_kd * (Tilt_Error - T_Error_ZminusOne)/T_dt;
							T_Error_ZminusOne = Tilt_Error;
						}
						else
						{
							T_Error_ZminusOne = 0;
							T_D = T_I = T_P = T_FooFighters = 0;  
						}

						E_CV = std::clamp<double>(E_P + E_I + E_D + E_FooFighters, -100, 100);
						T_CV = std::clamp<double>(T_P + T_I + T_D + T_FooFighters, -100, 100);
	#else					
						if(!E_PID_isDisabled)
						{
							// Elevator Motor PID
							this->E_FooFighters = std::copysign(ElevatePV, CalcHeight) * E_BiasTune;//(F_XYCurve<double>(ElevateHeightArray, yArrayFooFighters, CalcAngle, 10));
							this->E_P = Elevator_Error * E_kpTune;
							this->E_I = F_Limit(E_I_Min, E_I_Max, E_I+(E_kiTune * Elevator_Error *E_dt));
							this->E_D = E_kdTune * (Elevator_Error - E_Error_ZminusOne)/E_dt;
							E_Error_ZminusOne = Elevator_Error;
						}
						else
						{
							E_Error_ZminusOne = 0;
							E_FooFighters = E_D = E_I = E_P = 0;
						}


						if(!T_PID_isDisabled )
						{
							// Tilt Motor PID
							this->T_FooFighters = std::copysign(CalcAngle, TiltAngleSP) * T_BiasTune;
							this->T_P = Tilt_Error * T_kpTune;
							this->T_I = F_Limit(T_I_Min, T_I_Max, T_I+(T_kiTune * Tilt_Error *T_dt));
							this->T_D = T_kdTune * (Tilt_Error - T_Error_ZminusOne)/T_dt;
							T_Error_ZminusOne = Tilt_Error;
						}
						else
						{
							T_Error_ZminusOne = 0;
							T_D = T_I = T_P = T_FooFighters = 0;  
						}

						E_CV = std::clamp<double>(E_P + E_I + E_D + E_FooFighters, -100, 100);
						T_CV = std::clamp<double>(T_P + T_I + T_D + T_FooFighters, -100, 100);
	#endif
					}

					ElevateFalcon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,(E_CV/100.0));
					TiltFalcon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,(T_CV/100.0) * -1);

			//fmt::print("{}\n", Tilt_Error);
			E_ntPV->SetDouble(ElevatePV);
			E_ntCV->SetDouble(E_CV);
			E_ntP->SetDouble(E_P);
			E_ntI->SetDouble(E_I);
			E_ntD->SetDouble(E_D);
			E_ntErr->SetDouble(Elevator_Error);
			E_ntHeightLim->SetDouble(CalcHeight);

			T_ntPV->SetDouble(TiltPV);
			T_ntAnglePV->SetDouble(CalcAngle);
			T_ntCV->SetDouble(T_CV);
			T_ntP->SetDouble(T_P);
			T_ntI->SetDouble(T_I);
			T_ntD->SetDouble(T_D);
			T_ntErr->SetDouble(Tilt_Error);

			E_ntAtHome->SetBoolean(EHomeLS);
			T_ntAtHome->SetBoolean(THomeLS);
			T_ntAtMax->SetBoolean(TiltLimit->Get());

			E_ntPIDDisabled->SetBoolean(E_PID_isDisabled);
			T_ntPIDDisabled->SetBoolean(T_PID_isDisabled);
		}
	}
}

void X23_Elevator::InitNetworkTables()
{
#ifdef C_BUILD_OPT_ELEV_TUNING
    // Inputs
    E_ntKp = frc::Shuffleboard::GetTab("PID").Add("E_Kp", 1.1).WithWidget("Text Entry").GetEntry();
    E_ntKi = frc::Shuffleboard::GetTab("PID").Add("E_Ki", 0.9).WithWidget("Text Entry").GetEntry();
    E_ntKd = frc::Shuffleboard::GetTab("PID").Add("E_Kd", 3.0).WithWidget("Text Entry").GetEntry();
    E_ntBias = frc::Shuffleboard::GetTab("PID").Add("E_Bias", 0.7).WithWidget("Text Entry").GetEntry();
    E_ntSP = frc::Shuffleboard::GetTab("PID").Add("E_SP", 0.0).WithWidget("Text Entry").GetEntry();
    T_ntKp = frc::Shuffleboard::GetTab("PID").Add("T_Kp", 0.0).WithWidget("Text Entry").GetEntry();
    T_ntKi = frc::Shuffleboard::GetTab("PID").Add("T_Ki", 0.0).WithWidget("Text Entry").GetEntry();
    T_ntKd = frc::Shuffleboard::GetTab("PID").Add("T_Kd", 0.0).WithWidget("Text Entry").GetEntry();
    T_ntSP = frc::Shuffleboard::GetTab("PID").Add("T_SP", 0.0).WithWidget("Text Entry").GetEntry();
	T_ntAngleSP = frc::Shuffleboard::GetTab("PID").Add("T_AngleSP", 0.0).WithWidget("Text Entry").GetEntry();
	T_ntBias = frc::Shuffleboard::GetTab("PID").Add("T_FooFighters", 0.0).WithWidget("Text Entry").GetEntry();
#endif

    // Outputs 
    E_ntPV = frc::Shuffleboard::GetTab("PID").Add("E_PV", 0.0).WithWidget("Text Entry").GetEntry();
    E_ntCV = frc::Shuffleboard::GetTab("PID").Add("E_CV", 0.0).WithWidget("Text Entry").GetEntry();
    E_ntP = frc::Shuffleboard::GetTab("PID").Add("E_P", 0.0).WithWidget("Text Entry").GetEntry();
    E_ntI = frc::Shuffleboard::GetTab("PID").Add("E_I", 0.0).WithWidget("Text Entry").GetEntry();
    E_ntD = frc::Shuffleboard::GetTab("PID").Add("E_D", 0.0).WithWidget("Text Entry").GetEntry();
    E_ntErr = frc::Shuffleboard::GetTab("PID").Add("E_Err", 0.0).WithWidget("Text Entry").GetEntry();
    E_ntHeightLim = frc::Shuffleboard::GetTab("PID").Add("E_HeightLim", 0.0).WithWidget("Text Entry").GetEntry();

    T_ntPV = frc::Shuffleboard::GetTab("PID").Add("T_PV", 0.0).WithWidget("Text Entry").GetEntry();
    T_ntAnglePV = frc::Shuffleboard::GetTab("PID").Add("T_AnglePV", 0.0).WithWidget("Text Entry").GetEntry();
    T_ntCV = frc::Shuffleboard::GetTab("PID").Add("T_CV", 0.0).WithWidget("Text Entry").GetEntry();
    T_ntP = frc::Shuffleboard::GetTab("PID").Add("T_P", 0.0).WithWidget("Text Entry").GetEntry();
    T_ntI = frc::Shuffleboard::GetTab("PID").Add("T_I", 0.0).WithWidget("Text Entry").GetEntry();
    T_ntD = frc::Shuffleboard::GetTab("PID").Add("T_D", 0.0).WithWidget("Text Entry").GetEntry();
    T_ntErr = frc::Shuffleboard::GetTab("PID").Add("T_Err", 0.0).WithWidget("Text Entry").GetEntry();
    
    E_ntAtHome = frc::Shuffleboard::GetTab("X23").Add("Elev_IsAtHome", false).WithWidget("Boolean Box").GetEntry();
    T_ntAtHome = frc::Shuffleboard::GetTab("X23").Add("Tilt_IsAtHome", false).WithWidget("Boolean Box").GetEntry();
    T_ntAtMax = frc::Shuffleboard::GetTab("X23").Add("Tilt_IsAMax", false).WithWidget("Boolean Box").GetEntry();
    E_ntPIDDisabled = frc::Shuffleboard::GetTab("X23").Add("Elev_PID_IsDisabled", false).WithWidget("Boolean Box").GetEntry();
    T_ntPIDDisabled = frc::Shuffleboard::GetTab("X23").Add("Tilt_PID_IsDisabled", false).WithWidget("Boolean Box").GetEntry();
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
    if(TiltFalcon != nullptr) { TiltFalcon->Set(ControlMode::PercentOutput, 0.0); }
	T_CV = 0; 
	E_CV = 0;});
}

void X23_Elevator::ControlDirectElevate(double RawElevate)
{
	if(E_PID_isDisabled)
	{
		if(ElevateFalcon != nullptr) 
		{
			double ElevatePV = (ElevateFalcon->GetSelectedSensorPosition(0) * C_ELE_SCALE_FACTOR_POSN), Output = 0.0;
		
			if(ElevatorHome->Get() && (RawElevate > 0.0))
			{ 
				Output = 0.0;
			}
			else  
			{
				Output = RawElevate;
			}

			ElevateFalcon->Set(F_Limit(-1.0, 1.0, Output));

			E_ntPV->SetDouble(ElevatePV);
			E_ntAtHome->SetBoolean(ElevatorHome->Get()); 
			//E_ntPIDDisabled->SetBoolean(E_PID_isDisabled);
		}
	}
}

void X23_Elevator::ControlDirectTilt(double RawTiltFalcon)
{
	if(T_PID_isDisabled)
	{
		if(TiltFalcon != nullptr)
		{
			double Output = 0, TiltPV = 0.0;

			TiltPV = (TiltFalcon->GetSelectedSensorPosition(0) * C_TILT_SCALE_FACTOR_POSN);

			if(TiltHome->Get() && (RawTiltFalcon > 0.0))
			{ 
				Output = 0.0;
				TiltFalcon->SetSelectedSensorPosition(0);
			}
			else if(TiltLimit->Get() && (RawTiltFalcon < 0.0))
			{
				Output = 0.0;
			}
			else
			{
				Output  = RawTiltFalcon;
			}

			TiltFalcon->Set(ControlMode::PercentOutput, F_Limit(-0.5, 0.5, Output));

			T_ntPV->SetDouble(TiltPV);
			T_ntAtHome->SetBoolean(TiltHome->Get());
			T_ntAtMax->SetBoolean(TiltLimit->Get());
			
			T_ntPIDDisabled->SetBoolean(T_PID_isDisabled);
		}
	}
}

frc2::CommandPtr X23_Elevator::HybridZone()
{
	return frc2::cmd::RunOnce([this]{
								ElevatorHeightSP = 14.5;
								TiltAngleSP = 40;
								E_PID_isDisabled = false;
								T_PID_isDisabled = false;
								});
}
frc2::CommandPtr X23_Elevator::ConeOne()
{
	return frc2::cmd::RunOnce([this]{
								ElevatorHeightSP = 47.5;
								TiltAngleSP = 34;
								E_PID_isDisabled = false;
								T_PID_isDisabled = false;
								});	
}

frc2::CommandPtr X23_Elevator::ConeTwo()
{
	return frc2::cmd::RunOnce([this]{
								ElevatorHeightSP = 68.5;
								TiltAngleSP = 40;
								E_PID_isDisabled = false;
								T_PID_isDisabled = false;
								});
}

frc2::CommandPtr X23_Elevator::CubeOne()
{
	return frc2::cmd::RunOnce([this]{
								ElevatorHeightSP = 41.5;
								TiltAngleSP = 40;
								E_PID_isDisabled = false;
								T_PID_isDisabled = false;
								});
}

frc2::CommandPtr X23_Elevator::CubeTwo()
{
	return frc2::cmd::RunOnce([this]{
								ElevatorHeightSP = 68.5;
								TiltAngleSP = 42;
								E_PID_isDisabled = false;
								T_PID_isDisabled = false;
								});
}

frc2::CommandPtr X23_Elevator::Substation()
{
	return frc2::cmd::RunOnce([this]{
								ElevatorHeightSP = 15;
								TiltAngleSP = 38;
								E_PID_isDisabled = false;
								T_PID_isDisabled = false;
								});
}

frc2::CommandPtr X23_Elevator::HomePOS()
{
	return frc2::cmd::RunOnce([this]{
								ElevatorHeightSP = 0;
								TiltAngleSP = 0;
								E_PID_isDisabled = false;
								T_PID_isDisabled = false;
							});
}

bool X23_Elevator::IsAtHome()
{
    return this->atHome;
}

#include <fmt/format.h>

bool X23_Elevator::_arePointersValid()
{
	bool valid = true;

	if(TiltFalcon == nullptr) { fmt::print("{}\n", "Tilt Falcon pointer is null"); valid = false; }
	if(ElevateFalcon == nullptr) { fmt::print("{}\n", "Elevate Falcon pointer is null"); valid = false; }
	if(PincherSolenoid == nullptr) { fmt::print("{}\n", "Pincher Solenoid pointer is null"); valid = false; }
	if(TiltSolenoid == nullptr) { fmt::print("{}\n", "Claw Tilt Solenoid pointer is null"); valid = false; }
	if(ElevateBrake == nullptr) {fmt::print("{}\n", "Elevator Brake Solenoid pointer is null"); valid = false; }
	if(TiltLimit == nullptr) { fmt::print("{}\n", "Tilt Max Limit Switch pointer is null"); valid = false; }
	if(TiltHome == nullptr) { fmt::print("{}\n", "Tilt Home Limit Switch pointer is null"); valid = false; }
	if(ElevatorHome == nullptr) { fmt::print("{}\n", "Elevator Home Limit Switch pointer is null"); valid = false; }
	if(rTrigPinch == nullptr) {fmt::print("{}\n", "Pincher Input R_TRIG pointer is null"); valid = false; }
	if(rTrigEHome == nullptr) {fmt::print("{}\n", "Elevator Home LS R_TRIG pointer is null"); valid = false; }
	if(rTrigTHome == nullptr) {fmt::print("{}\n", "Tilt Home LS R_TRIG pointer is null"); valid = false; }
	if(rTrigTLimit == nullptr) {fmt::print("{}\n", "Tilt Max LS R_TRIG pointer is null"); valid = false; }
	if(DebouncePincher == nullptr) {fmt::print("{}\n", "Pincher Input Debouncer pointer is null"); valid = false; }
	if(DebounceTilt == nullptr) {fmt::print("{}\n", "Claw Tilt Input Debouncer pointer is null"); valid = false; }

#ifdef C_BUILD_OPT_ELEV_TUNING
	if(E_ntSP == nullptr) { fmt::print("{}\n", "Elevator NT SP pointer is null"); valid = false; }
	if(E_ntKp == nullptr) { fmt::print("{}\n", "Elevator NT Kp pointer is null"); valid = false; }
	if(E_ntKi == nullptr) { fmt::print("{}\n", "Elevator NT Ki pointer is null"); valid = false; }
	if(E_ntKd == nullptr) { fmt::print("{}\n", "Elevator NT Kd pointer is null"); valid = false; }
	if(E_ntBias == nullptr) { fmt::print("{}\n", "Elevator NT bias pointer is null"); valid = false; }

	if(T_ntSP == nullptr) { fmt::print("{}\n", "Tile NT SP pointer is null"); valid = false; }
	if(T_ntKp == nullptr) { fmt::print("{}\n", "Tile NT Kp pointer is null"); valid = false; }
	if(T_ntKi == nullptr) { fmt::print("{}\n", "Tile NT Ki pointer is null"); valid = false; }
	if(T_ntKd == nullptr) { fmt::print("{}\n", "Tile NT Kd pointer is null"); valid = false; }
#endif

	if(E_ntPV == nullptr) { fmt::print("{}\n", "Elevator NT PV pointer is null"); valid = false; }
	if(E_ntCV == nullptr) { fmt::print("{}\n", "Elevator NT VV pointer is null"); valid = false; }
	if(E_ntP == nullptr)  { fmt::print("{}\n", "Elevator NT P pointer is null"); valid = false; }
	if(E_ntI == nullptr)  { fmt::print("{}\n", "Elevator NT I pointer is null"); valid = false; }
	if(E_ntD == nullptr)  { fmt::print("{}\n", "Elevator NT D pointer is null"); valid = false; }
	if(E_ntErr == nullptr) { fmt::print("{}\n", "Elevator NT Error pointer is null"); valid = false; }

	if(T_ntPV == nullptr) { fmt::print("{}\n", "Tilt NT PV pointer is null"); valid = false; }
	if(T_ntCV == nullptr) { fmt::print("{}\n", "Tilt NT CV pointer is null"); valid = false; }
	if(T_ntP == nullptr)  { fmt::print("{}\n", "Tilt NT P pointer is null"); valid = false; }
	if(T_ntI == nullptr)  { fmt::print("{}\n", "Tilt NT I pointer is null"); valid = false; }
	if(T_ntD == nullptr)  { fmt::print("{}\n", "Tilt NT D pointer is null"); valid = false; }
	if(T_ntErr == nullptr) { fmt::print("{}\n", "Tilt NT Error pointer is null"); valid = false; }

	if(E_ntAtHome == nullptr) { fmt::print("{}\n", "Elvator NT Is At Home pointer is null"); valid = false; }
	if(T_ntAtHome == nullptr) { fmt::print("{}\n", "Tilt NT Is At Home pointer is null");valid = false; }
	if(E_ntPIDDisabled == nullptr) { fmt::print("{}\n", "Elevator NT PID Disabled pointer is null"); valid = false; }
	if(T_ntPIDDisabled == nullptr) { fmt::print("{}\n", "Tilt NT PID Disabled pointer is null"); valid = false; }
	if(T_ntAtMax == nullptr) { fmt::print("{}\n", "Tilt NT At Max Travel pointer is null"); valid = false; }

	return valid;
}