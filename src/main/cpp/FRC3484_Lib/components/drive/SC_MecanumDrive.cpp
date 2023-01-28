#include "FRC3484_Lib/components/drive/SC_MecanumDrive.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "units/math.h"

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

using namespace units;
using namespace frc;
using namespace SC;


SC::SC_MecanumDrive::SC_MecanumDrive()
{
    wheelSpeed_PV = wheelSpeed_SP = MecanumDriveWheelSpeeds{0_mps, 0_mps, 0_mps, 0_mps };
}

SC::SC_MecanumDrive::~SC_MecanumDrive()
{

}

void SC::SC_MecanumDrive::DriveCartesian(double X, double Y, double zRotation, degree_t gyro)
{
    // Correct gyro input to be between -pi and pi (-180 to 180 deg)
    radian_t phi = gyro >  180_deg ? gyro - 360_deg : gyro;
             phi = gyro < -180_deg ? gyro + 360_deg : gyro;

    // Initialize the input vector normalized against the configured
    // max wheel speed
    Translation2d velVec = Translation2d{units::meter_t{X}, units::meter_t{Y}}.RotateBy(Rotation2d{-gyro});

    // Vector2d velVec(X * maxWheelSpeed.to<double>(), 
    //                Y * maxWheelSpeed.to<double>());

    // Apply the gyro angle to rotate the velocity input to the robot's
    // coordinate plane
    // Vector rotation:
    // X' = X * cos(gyro) - Y * sin(gyro)
    // Y' = X * sin(gyro) + Y * cos(gyro)
    // Note: the Y is inverted in the above equations because the Y input
    // from the controller has +Y pointing downward
    //velVec.Rotate(gyro.value());

    double vel[4] = {
    /* FL */  velVec.X().value() + velVec.Y().value() + zRotation,
    /* FR */ -velVec.X().value() + velVec.Y().value() - zRotation,
    /* BL */ -velVec.X().value() + velVec.Y().value() + zRotation,
    /* BR */  velVec.X().value() + velVec.Y().value() - zRotation,
    };

    wheelSpeed_SP = MecanumDriveWheelSpeeds{
                        units::meters_per_second_t(vel[0]), 
                        units::meters_per_second_t(vel[1]),
                        units::meters_per_second_t(vel[2]),
                        units::meters_per_second_t(vel[3])
                    };
}

void SC::SC_MecanumDrive::DrivePolar(double radius, degree_t theta, double zRotation)
{
    DriveCartesian(radius * units::math::cos(theta), 
                    radius * units::math::sin(theta), 
                    zRotation, 0_deg);
}

void SC::SC_MecanumDrive::SetMaxWheelSpeed(units::meters_per_second_t mSpeed)
{
    this->maxWheelSpeed = mSpeed;
}

frc::MecanumDriveWheelSpeeds SC::SC_MecanumDrive::GetWheelSpeedsSetpoint()
{
    return wheelSpeed_SP;
}

frc::ChassisSpeeds SC::SC_MecanumDrive::GetChassisSpeed()
{
    return chassis;
}

double SC::SC_MecanumDrive::GetWheelOutput(SC::SC_Wheel wheelIdx)
{
    units::scalar_t out = 0;

    switch(wheelIdx)
    {
    case FRONT_LEFT:
        out = wheelSpeed_SP.frontLeft / maxWheelSpeed;
        break;
    case FRONT_RIGHT:
        out = wheelSpeed_SP.frontRight / maxWheelSpeed;
        break;
    case REAR_LEFT:
        out = wheelSpeed_SP.rearLeft / maxWheelSpeed;
        break;
    case REAR_RIGHT:
        out = wheelSpeed_SP.rearRight / maxWheelSpeed;
        break;
    default:
        out = 0;
        break;
    }

    return SC::F_Limit(-1.0, 1.0, out.to<double>());
}
