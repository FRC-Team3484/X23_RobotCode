#ifndef X23_Drivetrain_h
#define X23_Drivetrain_h

#include "FRC3484_Lib/components/drive/SC_MecanumDrive.h"
#include "frc/Solenoid.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h"

class X23_Drivetrain
{
public:
    X23_Drivetrain(std::tuple<int, int> chFR, 
                std::tuple<int, int> chFL, 
                std::tuple<int, int> chBR, 
                std::tuple<int, int> chBL,
                SC::SC_Solenoid ch_shift);

    ~X23_Drivetrain();

    void Drive(double direction_x, double direction_y, double rotation_z, double gyro, bool shift);

    void DriveAuto(double magnitude, double angle, double heading, bool shift);
    
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

    void SetDriveMode(SC::DriveMode dm);

    void StopMotors();

private:
    void _setOutputs();
    void _InitMotor(ctre::phoenix::motorcontrol::can::WPI_TalonSRX* Motor, bool Invert, ctre::phoenix::motorcontrol::can::WPI_TalonSRX* Master);

    SC::SC_MecanumDrive *md;
    frc::Solenoid *shifter;

    SC::DriveMode defaultMode = SC::DriveMode::MECANUM;
    SC::DriveMode activeMode = SC::DriveMode::DEFAULT;

    // Declare all four master controllers
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX *FR, *FR_Slave;
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX *FL, *FL_Slave;
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX *BR, *BR_Slave;
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX *BL, *BL_Slave;

};

#endif // DRIVETRAIN_H