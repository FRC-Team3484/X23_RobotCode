#include "Subsystems/X23_Drivetrain.h"

#include "FRC3484_Lib/utils/SC_Functions.h"

#include "Constants.h"
#include "tuple"
#include "Eigen/QR"
#include "frc/EigenCore.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Twist2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "wpimath/MathShared.h"

#include "units/angle.h"

using namespace SC;
using namespace frc;
using namespace units::length;
using namespace units::velocity;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::sensors;

X23_Drivetrain::X23_Drivetrain(std::tuple<int, int> chFR,
							   std::tuple<int, int> chFL,
							   std::tuple<int, int> chBR,
							   std::tuple<int, int> chBL,
							   SC_Solenoid ch_shift,
							   int PIGIMON)
{
	md = new SC::SC_MecanumKinematics(Translation2d{-13.8_in,7.5_in},Translation2d{13.8_in,7.5_in},Translation2d{-13.8_in,-7.5_in},Translation2d{13.8_in,-7.5_in});
	shifter = new Solenoid(ch_shift.CtrlID, ch_shift.CtrlType, ch_shift.Channel);
	Gyroscope = new Pigeon2(PIGIMON);
	dt_previousAngle = dtPose.Rotation();
	// this->dtPose(somestuffidk);
	dt_gyroOffset = dtPose.Rotation() - _GyroAngle();
	wpi::math::MathSharedStore::ReportUsage(wpi::math::MathUsageId::kOdometry_MecanumDrive, 1);

    int sCh = -1;
    // Initialize PreviousPOS
    Previous_WheelPOS.frontLeft  = units::make_unit<units::inch_t>(0);
    Previous_WheelPOS.frontRight = units::make_unit<units::inch_t>(0);
    Previous_WheelPOS.rearLeft   = units::make_unit<units::inch_t>(0);
    Previous_WheelPOS.rearRight  = units::make_unit<units::inch_t>(0);
    Gyroscope->SetYaw(0);
    Previous_Angle = _GyroAngle();


	// Initialize front right wheel
	if (chFR != C_BLANK_IDS)
	{
		FR = new WPI_TalonFX(std::get<0>(chFR));
		_InitMotor(FR, false, NULL);

		sCh = std::get<1>(chFR);
		if (sCh != C_DISABLED_CHANNEL)
		{
			FR_Slave = new WPI_TalonFX(sCh);
			_InitMotor(FR_Slave, false, FR);
		}
		else
		{
			FR_Slave = nullptr;
		}
	}
	else
	{
		FR = nullptr;
		FR_Slave = nullptr;
	}

	// Initialize front left wheel
	if (chFL != C_BLANK_IDS)
	{
		FL = new WPI_TalonFX(std::get<0>(chFL));
		_InitMotor(FL, true, NULL);

		sCh = std::get<1>(chFL);
		if (sCh != C_DISABLED_CHANNEL)
		{
			FL_Slave = new WPI_TalonFX(sCh);
			_InitMotor(FL_Slave, true, FL);
		}
		else
		{
			FL_Slave = nullptr;
		}
	}
	else
	{
		FL = nullptr;
		FL_Slave = nullptr;
	}

	// Initialize back right wheel
	if (chBR != C_BLANK_IDS)
	{
		BR = new WPI_TalonFX(std::get<0>(chBR));
		_InitMotor(BR, false, NULL);

		sCh = std::get<1>(chBR);
		if (sCh != C_DISABLED_CHANNEL)
		{
			BR_Slave = new WPI_TalonFX(sCh);
			_InitMotor(BR_Slave, false, BR);
		}
		else
		{
			BR_Slave = nullptr;
		}
	}
	else
	{
		BR = nullptr;
		BR_Slave = nullptr;
	}

	// Initialize back left wheel
	if (chBL != C_BLANK_IDS)
	{
		BL = new WPI_TalonFX(std::get<0>(chBL));
		_InitMotor(BL, true, NULL);

		sCh = std::get<1>(chBL);
		if (sCh != C_DISABLED_CHANNEL)
		{
			BL_Slave = new WPI_TalonFX(sCh);
			_InitMotor(BL_Slave, true, BL);
		}
		else
		{
			BL_Slave = nullptr;
		}
	}
	else
	{
		BL = nullptr;
		BL_Slave = nullptr;
	}
}

X23_Drivetrain::~X23_Drivetrain()
{
	if (md != nullptr) { delete md; md = nullptr; }
	if (shifter != nullptr) { delete shifter; shifter = nullptr; }

	if (FR != nullptr) { delete FR; FR = nullptr; }
	if (FL != nullptr) { delete FL; FL = nullptr; }
	if (BR != nullptr) { delete BR; BR = nullptr; }
	if (BL != nullptr) { delete BL; BL = nullptr; }

	if (FR_Slave != nullptr) { delete FR_Slave; FR_Slave = nullptr; }
	if (FL_Slave != nullptr) { delete FL_Slave; FL_Slave = nullptr; }
	if (BR_Slave != nullptr) { delete BR_Slave; BR_Slave = nullptr; }
	if (BL_Slave != nullptr) { delete BL_Slave; BL_Slave = nullptr; }
}

void X23_Drivetrain::Drive(double direction_x, double direction_y, double rotation_z, bool DriverOrient, bool shift)
{
	// octocanum shifter
	if (shifter != nullptr)
		shifter->Set(shift);

	if (md != nullptr)
	{
		if (shift)
		{
			direction_x = 0;
		}
		if (Gyroscope != nullptr)
		{
			md->DriveCartesian(direction_x, direction_y, rotation_z,
							   DriverOrient ? units::make_unit<units::degree_t>(Gyroscope->GetYaw()) : 0_deg);
		}
		else
		{
			md->DriveCartesian(direction_x, direction_y, rotation_z, 0_deg);
		}
		_setOutputs();
	}
	else
	{
		this->StopMotors();
	}
}

void X23_Drivetrain::DriveAuto(double magnitude, double angle, double heading, bool shift)
{
	if (shifter != nullptr)
		shifter->Set(shift);

	if (md != nullptr)
	{
		md->DrivePolar(magnitude,
					   units::make_unit<units::degree_t>(angle),
					   heading);

		_setOutputs();
	}
}

void X23_Drivetrain::DriveAuto(frc::ChassisSpeeds Speeds)
{
if (md != nullptr)
	{
		md->DriveCartesian( Speeds.vx.to<double>(), Speeds.vy.to<double>(), Speeds.omega.to<double>(),
		0_deg);
		
		_setOutputs();
	}
}
void X23_Drivetrain::_InitMotor(WPI_TalonFX *Motor, bool Invert, WPI_TalonFX *Master)
void X23_Drivetrain::_InitMotor(WPI_TalonFX *Motor, bool Invert, WPI_TalonFX *Master)
{
	if (Motor != NULL)
	{
		Motor->SetInverted(Invert);
		Motor->SetNeutralMode(NeutralMode::Brake);
		Motor->ConfigOpenloopRamp(0);
		Motor->ConfigClosedloopRamp(0);
		Motor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
		Motor->SetSelectedSensorPosition(0);

		if (Master != NULL)
		{
			Motor->Follow(*Master);
		}
	}
}

void X23_Drivetrain::DriveDirect(double rawFR, double rawFL, double rawBR, double rawBL)
{
	if (FR != nullptr)	{ FR->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, rawFR)); }
	if (FL != nullptr)	{ FL->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, rawFL)); }
	if (BR != nullptr)  { BR->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, rawBR)); }
	if (BL != nullptr)  { BL->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, rawBL)); }
}

void X23_Drivetrain::SetPose(frc::Pose2d &NewPose)
{
	this->dtPose = NewPose;
}

void X23_Drivetrain::UpdateOdometry(MecanumDriveWheelPositions& wheelPositions) 
{
    Rotation2d angle = _GyroAngle() + dt_gyroOffset;

    MecanumDriveWheelPositions wheelDeltas{
      wheelPositions.frontLeft - Previous_WheelPOS.frontLeft,
      wheelPositions.frontRight - Previous_WheelPOS.frontRight,
      wheelPositions.rearLeft - Previous_WheelPOS.rearLeft,
      wheelPositions.rearRight - Previous_WheelPOS.rearRight,
  };

  Twist2d twist = this->md->ToTwist2d(wheelDeltas);
  twist.dtheta = (angle - Previous_Angle).Radians();

  Pose2d newPose = dtPose.Exp(twist);

  Previous_Angle = angle;
  Previous_WheelPOS = wheelPositions;
  dtPose = {newPose.Translation(), angle};
}


frc::Pose2d X23_Drivetrain::GetPose()
{
	return dtPose;
}

Rotation2d X23_Drivetrain::_GyroAngle()
{
	if (Gyroscope != nullptr)
	{
		return Rotation2d{units::make_unit<units::degree_t>(Gyroscope->GetYaw())};
	}
	else
	{
		return Rotation2d{0_deg};
	}
}

MecanumDriveWheelPositions X23_Drivetrain::_GetdtPOS()
{
	if (FR != nullptr && BR != nullptr && FL != nullptr && BL != nullptr)
	{
		return MecanumDriveWheelPositions{
			units::make_unit<meter_t>(FL->GetSelectedSensorPosition() * C_DT_SCALE_FACTOR),
			units::make_unit<meter_t>(FR->GetSelectedSensorPosition() * C_DT_SCALE_FACTOR),
			units::make_unit<meter_t>(BL->GetSelectedSensorPosition() * C_DT_SCALE_FACTOR),
			units::make_unit<meter_t>(BR->GetSelectedSensorPosition() * C_DT_SCALE_FACTOR)};
	}
	else
	{
		return MecanumDriveWheelPositions{
			0_m, 0_m, 0_m, 0_m};
	}
}

void X23_Drivetrain::StopMotors()
{
	if (FR != nullptr)
	{
		FR->Set(ControlMode::PercentOutput, 0.0);
	}
	if (FL != nullptr)
	{
		FL->Set(ControlMode::PercentOutput, 0.0);
	}
	if (BR != nullptr)
	{
		BR->Set(ControlMode::PercentOutput, 0.0);
	}
	if (BL != nullptr)
	{
		BL->Set(ControlMode::PercentOutput, 0.0);
	}
}

void X23_Drivetrain::SetBrakeMode()
{
	if (this->FR != NULL)
	{
		this->FR->SetNeutralMode(NeutralMode::Brake);
	}
	if (this->FL != NULL)
	{
		this->FL->SetNeutralMode(NeutralMode::Brake);
	}
	if (this->BR != NULL)
	{
		this->BR->SetNeutralMode(NeutralMode::Brake);
	}
	if (this->BL != NULL)
	{
		this->BL->SetNeutralMode(NeutralMode::Brake);
	}

	if (this->FR_Slave != nullptr)
	{
		FR_Slave->SetNeutralMode(NeutralMode::Brake);
	}
	if (this->FL_Slave != nullptr)
	{
		FL_Slave->SetNeutralMode(NeutralMode::Brake);
	}
	if (this->BR_Slave != nullptr)
	{
		BR_Slave->SetNeutralMode(NeutralMode::Brake);
	}
	if (this->BL_Slave != nullptr)
	{
		BL_Slave->SetNeutralMode(NeutralMode::Brake);
	}
}

void X23_Drivetrain::SetCoastMode()
{
	if (this->FR != NULL)
	{
		this->FR->SetNeutralMode(NeutralMode::Coast);
	}
	if (this->FL != NULL)
	{
		this->FL->SetNeutralMode(NeutralMode::Coast);
	}
	if (this->BR != NULL)
	{
		this->BR->SetNeutralMode(NeutralMode::Coast);
	}
	if (this->BL != NULL)
	{
		this->BL->SetNeutralMode(NeutralMode::Coast);
	}

	if (this->FR_Slave != nullptr)
	{
		FR_Slave->SetNeutralMode(NeutralMode::Coast);
	}
	if (this->FL_Slave != nullptr)
	{
		FL_Slave->SetNeutralMode(NeutralMode::Coast);
	}
	if (this->BR_Slave != nullptr)
	{
		BR_Slave->SetNeutralMode(NeutralMode::Coast);
	}
	if (this->BL_Slave != nullptr)
	{
		BL_Slave->SetNeutralMode(NeutralMode::Coast);
	}
}

void X23_Drivetrain::_setOutputs()
{
	if (md != nullptr)
	{
		if (FR != nullptr)
		{
			FR->Set(ControlMode::PercentOutput, md->GetWheelOutput(FRONT_RIGHT));
		}
		if (FL != nullptr)
		{
			FL->Set(ControlMode::PercentOutput, md->GetWheelOutput(FRONT_LEFT));
		}
		if (BR != nullptr)
		{
			BR->Set(ControlMode::PercentOutput, md->GetWheelOutput(REAR_RIGHT));
		}
		if (BL != nullptr)
		{
			BL->Set(ControlMode::PercentOutput, md->GetWheelOutput(REAR_LEFT));
		}
	}
}