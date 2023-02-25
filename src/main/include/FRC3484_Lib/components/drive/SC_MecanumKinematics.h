#ifndef SC_MECANUMDRIVE_H
#define SC_MECANUMDRIVE_H

#include <span>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include "frc/geometry/Twist2d.h"
#include <frc/geometry/Translation2d.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include "frc/kinematics/MecanumDriveWheelPositions.h"
#include "frc/kinematics/MecanumDriveWheelSpeeds.h"
// #include "Eigen/QR"
// #include "frc/EigenCore.h"
// #include "wpimath/MathShared.h"

#include "units/angle.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "Eigen/QR"
#include "frc/EigenCore.h"
#include "frc/geometry/Twist2d.h"
#include "frc/kinematics/MecanumDriveWheelPositions.h"
#include "frc/kinematics/MecanumDriveWheelSpeeds.h"
#include "wpimath/MathShared.h"
namespace SC
{
	class SC_MecanumKinematics : public wpi::Sendable,
								 public wpi::SendableHelper<SC_MecanumKinematics>
	{
	public:
		struct SC_MD_WheelSpeeds
		{
			double WS_FrontLeft = 0.0;
			double WS_FrontRight = 0.0;
			double WS_BackLeft = 0.0;
			double WS_BackRight = 0.0;
		};
frc::Twist2d ToTwist2d(const frc::MecanumDriveWheelPositions& wheelDeltas) const;

		SC_MecanumKinematics();

		~SC_MecanumKinematics();

        /**
         * @brief   DriveCartesian
         */
        void DriveCartesian(double X, double Y, double zRotation, units::angle::degree_t gyro);
        void SetInverseKinematics(Translation2d fl,Translation2d fr,Translation2d rl,Translation2d rr);  
        /**
         * @brief   DrivePolar
         */
        void DrivePolar(double radius, units::angle::degree_t theta, double zRotation);

		SC_MD_WheelSpeeds GetWheelSpeedsSetpoint();

		double GetWheelOutput(SC::SC_Wheel wheelIdx);

		/*
		  Inherited Function Overrides
		*/

		void InitSendable(wpi::SendableBuilder &builder) override;

frc::MecanumDriveWheelSpeeds  SC_MecanumKinematics::ToWheelSpeeds(
    const frc::ChassisSpeeds& chassisSpeeds,
    const frc::Translation2d& centerOfRotation = frc::Translation2d{}) const;
   
    private:
        SC_MD_WheelSpeeds wheelSpeed_SP;
        Translation2d PreviousCoR;
  mutable frc::Matrixd<4, 3> inverseKinematics;
    Eigen::HouseholderQR<frc::Matrixd<4, 3>> forwardKinematics;
    frc::Translation2d frontLeftWheel;
    frc::Translation2d frontRightWheel;
    frc::Translation2d rearLeftWheel;
    frc::Translation2d rearRightWheel;

		void _desaturateWheelSpeeds(std::span<double> speeds);
	};
}

#endif