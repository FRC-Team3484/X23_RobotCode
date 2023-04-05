#ifndef X23_Drivetrain_h
#define X23_Drivetrain_h

#include "FRC3484_Lib/components/drive/SC_MecanumKinematics.h"

// #include <frc/kinematics/MecanumDriveOdometry.h>
#include "frc/kinematics/MecanumDriveWheelSpeeds.h"
#include "frc/kinematics/MecanumDriveWheelPositions.h"

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc2/command/SubsystemBase.h"
#include "frc/Solenoid.h"

#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/sensors/Pigeon2.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include "networktables/NetworkTableInstance.h"
#include <networktables/GenericEntry.h>

class X23_Drivetrain : public frc2::SubsystemBase
{
public:
    X23_Drivetrain(std::tuple<int, int> chFR, 
                std::tuple<int, int> chFL, 
                std::tuple<int, int> chBR, 
                std::tuple<int, int> chBL,
                SC::SC_Solenoid ch_shift,
                int PIGIMON);

    ~X23_Drivetrain();

    void Drive(double direction_x, double direction_y, double rotation_z, bool DriverOrient, bool shift);

    void DriveAuto(double magnitude, double angle, double heading, bool shift);
    void DriveAuto(frc::ChassisSpeeds Speeds);
    
    /**
     * @brief   Forces motors to drive with fixed values. Inputs are assumed
     *          to be percent output values between -1.0 and 1.0.
     * 
     *          All input values are internally limited before being assigned
     *          to the motor controllers.
     */
    void DriveDirect(double rawFR, double rawFL, double rawBR, double rawBL);

    /**
     * @brief   Ramp is used to control the slew rate (acceleration)
     *          of the drivetrain.
     */
    void Ramp();

    double GetDistance();

    void ResetGyro();

    void SetPose(frc::Pose2d &NewPose);
    void SetPose(double angle);

    void UpdateOdometry(frc::MecanumDriveWheelPositions& wheelPositions);

    void ToWheelSpeeds (frc::ChassisSpeeds& speeds);

    frc::Pose2d GetPose();
    
    void ResetPose();
    
    void StopMotors();
	/**
	 * @brief	Switches all motor controllers to brake mode
	*/
	void SetBrakeMode();

	/**
	 * @brief	Switches all motor controllers to coast mode
	*/
	void SetCoastMode();

private:
    void _setOutputs();
    void _InitMotor(ctre::phoenix::motorcontrol::can::WPI_TalonFX* Motor, bool Invert, ctre::phoenix::motorcontrol::can::WPI_TalonFX* Master);
    
    frc::Rotation2d _GyroAngle();
    frc::MecanumDriveWheelPositions _GetdtPOS();
    
    SC::SC_MecanumKinematics *md;
    frc::Solenoid *shifter;

    frc::Pose2d dtPose;

    double GyroArray [3];

    ctre::phoenix::sensors::Pigeon2 *Gyroscope;

    frc::MecanumDriveWheelPositions dt_previousWheelPositions;
    
    frc::Rotation2d dt_previousAngle;
    frc::Rotation2d dt_gyroOffset;

    // Declare all four master controllers
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *FR, *FR_Slave;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *FL, *FL_Slave;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *BR, *BR_Slave;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *BL, *BL_Slave;

    frc::MecanumDriveWheelPositions Previous_WheelPOS;
    nt::GenericEntry *Gyro_Angle;
};

#endif // DRIVETRAIN_H
