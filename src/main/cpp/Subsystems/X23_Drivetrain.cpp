#include "Subsystems/X23_Drivetrain.h"
#include "units/angle.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "Constants.h"
#include "tuple"

using namespace SC;
using namespace frc;
using namespace units::length;
using namespace units::velocity;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


X23_Drivetrain::X23_Drivetrain(std::tuple<int, int> chFR, 
                    			std::tuple<int, int> chFL, 
                    			std::tuple<int, int> chBR, 
                    			std::tuple<int, int> chBL, 
                    			SC_Solenoid ch_shift)
{
    md = new SC::SC_MecanumKinematics();
    shifter = new Solenoid(ch_shift.CtrlID, ch_shift.CtrlType, ch_shift.Channel);

    int sCh = -1;

    // Initialize front right wheel
    if(chFR != C_BLANK_IDS) 
    { 
        FR = new WPI_TalonSRX(std::get<0>(chFR));
		_InitMotor(FR, true, NULL);

        sCh = std::get<1>(chFR);
        if(sCh != C_DISABLED_CHANNEL) { FR_Slave = new WPI_TalonSRX(sCh); _InitMotor(FR_Slave, true, FR); }
		else { FR_Slave = nullptr; }
    } 
    else
    {
        FR = nullptr;
        FR_Slave = nullptr; 
    }


    // Initialize front left wheel
    if(chFL != C_BLANK_IDS) 
    { 
        FL = new WPI_TalonSRX(std::get<0>(chFL));
		_InitMotor(FL, false, NULL);

        sCh = std::get<1>(chFL);
        if(sCh != C_DISABLED_CHANNEL) { FL_Slave = new WPI_TalonSRX(sCh); _InitMotor(FL_Slave, false, FL); }
		else { FL_Slave = nullptr; }
    }
    else
    {
        FL = nullptr;
        FL_Slave = nullptr;
    }

    // Initialize back right wheel
    if(chBR != C_BLANK_IDS) 
    { 
        BR = new WPI_TalonSRX(std::get<0>(chBR));
		_InitMotor(BR, true, NULL);

        sCh = std::get<1>(chBR);
        if(sCh != C_DISABLED_CHANNEL) { BR_Slave = new WPI_TalonSRX(sCh); _InitMotor(BR_Slave, true, BR); }
		else { BR_Slave = nullptr; }
    } 
    else
    {
        BR = nullptr;
        BR_Slave = nullptr;
    }

    // Initialize back left wheel
    if(chBL != C_BLANK_IDS) 
    { 
        BL = new WPI_TalonSRX(std::get<0>(chBL));
		_InitMotor(BL, false, NULL);

        sCh = std::get<1>(chBL);
        if(sCh != C_DISABLED_CHANNEL) { BL_Slave = new WPI_TalonSRX(sCh); _InitMotor(BL_Slave, false, BL); }
		else { BL_Slave = nullptr; }
    } 
    else
    {
        BL = nullptr;
        BL_Slave = nullptr;
    }

    if(FR != nullptr) { FR->SetNeutralMode(NeutralMode::Coast); }
    if(FL != nullptr) { FL->SetNeutralMode(NeutralMode::Coast); }
    if(BR != nullptr) { BR->SetNeutralMode(NeutralMode::Coast); }
    if(BL != nullptr) { BL->SetNeutralMode(NeutralMode::Coast); }

    if(FR_Slave != nullptr) { FR_Slave->SetNeutralMode(NeutralMode::Coast); }
    if(FL_Slave != nullptr) { FL_Slave->SetNeutralMode(NeutralMode::Coast); }
    if(BR_Slave != nullptr) { BR_Slave->SetNeutralMode(NeutralMode::Coast); }
    if(BL_Slave != nullptr) { BL_Slave->SetNeutralMode(NeutralMode::Coast); }
}

X23_Drivetrain::~X23_Drivetrain()
{
    if(md != nullptr) { delete md; md = nullptr; }
    if(shifter != nullptr) { delete shifter; shifter = nullptr; }

    if(FR != nullptr) { delete FR; FR = nullptr; }
    if(FL != nullptr) { delete FL; FL = nullptr; }
    if(BR != nullptr) { delete BR; BR = nullptr; }
    if(BL != nullptr) { delete BL; BL = nullptr; }
    if(FR_Slave != nullptr) { delete FR_Slave; FR_Slave = nullptr; }
    if(FL_Slave != nullptr) { delete FL_Slave; FL_Slave = nullptr; }
    if(BR_Slave != nullptr) { delete BR_Slave; BR_Slave = nullptr; }
    if(BL_Slave != nullptr) { delete BL_Slave; BL_Slave = nullptr; }

}

void X23_Drivetrain::Drive(double direction_x, double direction_y, double rotation_z, double gyro, bool shift)
{
    // octocanum shifter
    if(shifter != nullptr)
        shifter->Set(shift);

    if(md != nullptr)
    {
        if (shift)
        {
            direction_x = 0;
        }

        md->DriveCartesian(direction_x, direction_y, rotation_z, 
                            units::make_unit<units::degree_t>(gyro));
        
        _setOutputs();
    }
    else
    {
        this->StopMotors();
    }    
}

void X23_Drivetrain::DriveAuto(double magnitude, double angle, double heading, bool shift)
{
    if(shifter != nullptr)
        shifter->Set(shift);

    if(md != nullptr)
    {
        md->DrivePolar(magnitude,
                    	units::make_unit<units::degree_t>(angle), 
                    	heading);

        _setOutputs();
    }
}

void X23_Drivetrain::_InitMotor(WPI_TalonSRX* Motor, bool Invert, WPI_TalonSRX* Master){
    if(Motor != NULL)
	{
		Motor->SetInverted(Invert);
		Motor->SetNeutralMode(NeutralMode::Brake);
		Motor->ConfigOpenloopRamp(2);
		Motor->ConfigClosedloopRamp(0);
		Motor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
		Motor->SetSelectedSensorPosition(0);

		if(Master != NULL) { Motor->Follow(*Master); }
	}
}

void X23_Drivetrain::DriveDirect(double rawFR, double rawFL, double rawBR, double rawBL)
{
    if(FR != nullptr) { FR->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, rawFR)); }
    if(FL != nullptr) { FL->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, rawFL)); }
    if(BR != nullptr) { BR->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, rawBR)); }
    if(BL != nullptr) { BL->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, rawBL)); }
}

void X23_Drivetrain::StopMotors()
{
    if(FR != nullptr) { FR->Set(ControlMode::PercentOutput, 0.0); }
    if(FL != nullptr) { FL->Set(ControlMode::PercentOutput, 0.0); }
    if(BR != nullptr) { BR->Set(ControlMode::PercentOutput, 0.0); }
    if(BL != nullptr) { BL->Set(ControlMode::PercentOutput, 0.0); }
}

void X23_Drivetrain::SetBrakeMode()
{
	
}

void X23_Drivetrain::SetCoastMode()
{

}

void X23_Drivetrain::_setOutputs()
{
    if(md != nullptr)
    {
        if(FR != nullptr) { FR->Set(ControlMode::PercentOutput, md->GetWheelOutput(FRONT_RIGHT)); }
        if(FL != nullptr) { FL->Set(ControlMode::PercentOutput, md->GetWheelOutput(FRONT_LEFT)); }
        if(BR != nullptr) { BR->Set(ControlMode::PercentOutput, md->GetWheelOutput(REAR_RIGHT)); }
        if(BL != nullptr) { BL->Set(ControlMode::PercentOutput, md->GetWheelOutput(REAR_LEFT)); }
    }
}