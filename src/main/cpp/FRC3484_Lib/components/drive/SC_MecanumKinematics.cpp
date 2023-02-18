#include "FRC3484_Lib/components/drive/SC_MecanumKinematics.h"

#include "wpi/sendable/SendableBuilder.h"
#include "wpi/sendable/SendableRegistry.h"
#include "Eigen/QR"
#include "frc/EigenCore.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Twist2d.h"
#include "units/math.h"
#include "units/dimensionless.h"

#include "FRC3484_Lib/utils/SC_Functions.h"


using namespace frc;

SC::SC_MecanumKinematics::SC_MecanumKinematics()
{
    wheelSpeed_SP = SC_MD_WheelSpeeds{0.0, 0.0, 0.0, 0.0};
}

SC::SC_MecanumKinematics::~SC_MecanumKinematics()
{

}

void SC::SC_MecanumKinematics::DriveCartesian(double X, double Y, double zRotation, units::degree_t gyro)
{
    // Correct gyro input to be between -pi and pi (-180 to 180 deg)
    //radian_t phi = gyro >  180_deg ? gyro - 360_deg : gyro;
    //         phi = gyro < -180_deg ? gyro + 360_deg : gyro;

    // Initialize the input vector normalized against the configured
    // max wheel speed
    Translation2d velVec = Translation2d{units::meter_t{X}, units::meter_t{Y}}.RotateBy(Rotation2d{-gyro});

    // Apply the gyro angle to rotate the velocity input to the robot's
    // coordinate plane
    // Vector rotation:
    // X' = X * cos(gyro) - Y * sin(gyro)
    // Y' = X * sin(gyro) + Y * cos(gyro)
    // Note: the Y is inverted in the above equations because the Y input
    // from the controller has +Y pointing downward

    double vel[4] = {
    /* FL */  velVec.X().value() + velVec.Y().value() + zRotation,
    /* FR */ -velVec.X().value() + velVec.Y().value() - zRotation,
    /* BL */ -velVec.X().value() + velVec.Y().value() + zRotation,
    /* BR */  velVec.X().value() + velVec.Y().value() - zRotation,
    };

	this->_desaturateWheelSpeeds(vel);

	this->wheelSpeed_SP = {vel[FRONT_LEFT], vel[FRONT_RIGHT], vel[REAR_LEFT], vel[REAR_RIGHT]};
}

void SC::SC_MecanumKinematics::DrivePolar(double radius, units::degree_t theta, double zRotation)
{
    DriveCartesian(radius * Rotation2d{theta}.Cos(), 
                	radius * Rotation2d{theta}.Sin(), 
                    zRotation, 0_deg);
}

SC::SC_MecanumKinematics::SC_MD_WheelSpeeds SC::SC_MecanumKinematics::GetWheelSpeedsSetpoint()
{
    return wheelSpeed_SP;
}

double SC::SC_MecanumKinematics::GetWheelOutput(SC::SC_Wheel wheelIdx)
{
	double out = 0;

    switch(wheelIdx)
    {
    case FRONT_LEFT:
        out = wheelSpeed_SP.WS_FrontLeft;
        break;
    case FRONT_RIGHT:
        out = wheelSpeed_SP.WS_FrontRight;
        break;
    case REAR_LEFT:
        out = wheelSpeed_SP.WS_BackLeft;
        break;
    case REAR_RIGHT:
        out = wheelSpeed_SP.WS_BackRight;
        break;
    default:
        out = 0;
        break;
    }

	return SC::F_Limit(-1.0, 1.0, out);
}

void SC::SC_MecanumKinematics::InitSendable(wpi::SendableBuilder &builder)
{
	builder.SetSmartDashboardType("SC_MecanumKinematics");
	builder.SetActuator(true);
	builder.SetSafeState([=,this] { DriveCartesian(0, 0, 0, 0_deg); } );
	builder.AddDoubleProperty("FL", [=,this] { return wheelSpeed_SP.WS_FrontLeft; }, 	[=,this](double val) { wheelSpeed_SP.WS_FrontLeft = val;});
	builder.AddDoubleProperty("FR", [=,this] { return wheelSpeed_SP.WS_FrontRight; }, 	[=,this](double val) { wheelSpeed_SP.WS_FrontRight = val;});
	builder.AddDoubleProperty("BL", [=,this] { return wheelSpeed_SP.WS_BackLeft; }, 	[=,this](double val) { wheelSpeed_SP.WS_BackLeft = val;});
	builder.AddDoubleProperty("BR", [=,this] { return wheelSpeed_SP.WS_BackRight; }, 	[=,this](double val) { wheelSpeed_SP.WS_BackRight = val;});
}

Twist2d MecanumDriveKinematics::ToTwist2d(
    const MecanumDriveWheelPositions& wheelDeltas) const {
  Vectord<4> wheelDeltasVector{
      wheelDeltas.frontLeft.value(), wheelDeltas.frontRight.value(),
      wheelDeltas.rearLeft.value(), wheelDeltas.rearRight.value()};

  Eigen::Vector3d twistVector = m_forwardKinematics.solve(wheelDeltasVector);

  return {units::meter_t{twistVector(0)},  // NOLINT
          units::meter_t{twistVector(1)}, units::radian_t{twistVector(2)}};
}


/*===================*/
/* Private Functions */
/*===================*/

void SC::SC_MecanumKinematics::_desaturateWheelSpeeds(std::span<double> speeds)
{
	double maxMag = std::abs(speeds[0]);

	for (size_t i = 1; i < speeds.size(); i++)
	{
		double temp = std::abs(speeds[i]);
		if(maxMag < temp) { maxMag = temp; }
	}

	if(maxMag > 1.0) { for(size_t i = 0; i < speeds.size(); i++) { speeds[i] /= maxMag; } }
}