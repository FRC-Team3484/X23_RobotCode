#ifndef SC_MECANUMDRIVE_H
#define SC_MECANUMDRIVE_H

#include <span>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include <frc/geometry/Translation2d.h>

#include "units/angle.h"

#include "FRC3484_Lib/utils/SC_Datatypes.h"

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

        SC_MecanumKinematics();

        ~SC_MecanumKinematics();

        /**
         * @brief   DriveCartesian
         */
        void DriveCartesian(double X, double Y, double zRotation, units::angle::degree_t gyro);

        /**
         * @brief   DrivePolar
         */
        void DrivePolar(double radius, units::angle::degree_t theta, double zRotation);

        SC_MD_WheelSpeeds GetWheelSpeedsSetpoint();

        double GetWheelOutput(SC::SC_Wheel wheelIdx);
		
		/*
			Inherited Function Overrides
		*/

		void InitSendable(wpi::SendableBuilder& builder) override;

    private:
        SC_MD_WheelSpeeds wheelSpeed_SP;

		void _desaturateWheelSpeeds(std::span<double> speeds);
    };
}

#endif