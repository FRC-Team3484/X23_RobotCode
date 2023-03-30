#pragma once
#include <tuple>
#include <map>
#include <unordered_map>
#include <array>
#include <iostream>
#include "FRC3484_Lib/utils/SC_Functions.h"
// #include <units/acceleration.h>
// #include <units/velocity.h>

#include <frc/smartdashboard/SendableChooser.h>
// #include <frc/DriverStation.h>
// #include <frc/geometry/Pose2d.h>
// #include <frc/geometry/Translation2d.h>
#include <frc2/command/PrintCommand.h>

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/WaitCommand.h>
// #include <frc2/command/button/CommandXboxController.h>
// #include <frc2/command/button/CommandPS4Controller.h>

#include "Constants.h"
#include "Subsystems/X23_Drivetrain.h"
#include "Subsystems/X23_Intake.h"
#include "Subsystems/X23_Elevator.h"

#define FEILD_WIDTH 16.53_m
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer 
{
 public:
  RobotContainer();

  void setAutoDefaults();
  void setTeleopDefaults();

  void startAutoCommand();
  void endAutoCommand();
  frc::SendableChooser<frc2::Command*> autonomousChooser;

  X23_Drivetrain _drivetrain{ std::make_tuple<int, int>(C_FX_FL_MASTER, C_FX_FL_SLAVE),
                              std::make_tuple<int, int>(C_FX_FR_MASTER, C_FX_FR_SLAVE),
								              std::make_tuple<int, int>(C_FX_BL_MASTER, C_FX_BL_SLAVE),
                              std::make_tuple<int, int>(C_FX_BR_MASTER, C_FX_BR_SLAVE),
                              SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::REVPH, C_DRIVE_SOL}, C_PIGEON_IMU};

  X23_Intake _intake{C_SPX_INTAKE};

  X23_Elevator _elevator{ C_FX_ELEVATEMOTOR,
                          C_FX_TILTMOTOR,
                          SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::REVPH, C_SOL_CLAW_GRIP},
                          SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::REVPH, C_SOL_CLAW_TILT},
                          SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::REVPH, C_SOL_ELEVATOR_BRAKE},
                          C_DI_CH_ELEVATOR_TILT_HOME,
                          C_DI_CH_ELEVATOR_HOME, 
                          C_DI_CH_ELEVATOR_TILT_MAX};
 private: 
  void ConfigureBindings(); 

  /**************
   * Subsystems *
   **************/


  // List of possible autos and relevant configs.
  std::array<std::string, 10> autoNames {
    {
      "NO AUTO",
      "Mobility",
      "Balance", 
    }
  };

  /****************
   * Auto Chooser *
   ****************/
  frc2::CommandPtr NoAutoCommand = frc2::PrintCommand("NO AUTO\n").ToPtr();

  frc2::CommandPtr MobilityAutoCommand = frc2::cmd::Sequence(frc2::FunctionalCommand(
					  			// OnInit
				  				[this] {}, 
						  		// OnExecute
                  [this] { _intake.Eject_Cone(); },
                   // OnEnd
                  [this](bool interrupted) { _intake.StopIntake(); },
                  // IsFinished
                  [this] { return true;},
                  {&_intake}
                  ).ToPtr(),
                  frc2::WaitCommand(2.0_s).ToPtr(),
                  frc2::FunctionalCommand(
                  //startup
                  [this] {},
                  //execute
							  	[this] { _drivetrain.Drive(0, 0.25, 0, false, true); },
							  	// Interrupted Function
							  	[this](bool interrupted) { _drivetrain.Drive(0.0, 0.0, 0.0, false, true); }, 
							  	// End Condition
							  	[this] {return _drivetrain.GetDistance() < 15.0;},
							  	{&_drivetrain}
							  	).ToPtr());

  frc2::CommandPtr BalanceAutoCommand = frc2::cmd::Sequence(frc2::FunctionalCommand(
					  			// OnInit
				  				[this] {}, 
						  		// OnExecute
                  [this] { _intake.Eject_Cone(); },
                   // OnEnd
                  [this](bool interrupted) { _intake.StopIntake(); },
                  // IsFinished
                  [this] { return true;},
                  {&_intake}
                  ).ToPtr(),
                  frc2::WaitCommand(2.0_s).ToPtr(),
                  frc2::FunctionalCommand(
                  //startup
                  [this] {},
                  //execute
							  	[this] { _drivetrain.Drive(0, 0.25, 0, false, true); },
							  	// Interrupted Function
							  	[this](bool interrupted) { _drivetrain.Drive(0.0, 0.0, 0.0, false, true); }, 
							  	// End Condition
							  	[this] {return _drivetrain.GetDistance() < 15.0;},
							  	{&_drivetrain}
							  	).ToPtr(),
                  frc2::FunctionalCommand(
                  // Startup
				  				[this] {},
                  // Execute
							  	[this] { _drivetrain.Drive(0, -0.25, 0, false, true); },
                  // Interrupted Function
							  	[this](bool interrupted) { _drivetrain.Drive(0.0, 0.0, 0.0, false, true); }, 
                  // End Condition
							  	[this] {return _drivetrain.GetDistance() > 6.0;},
                  {&_drivetrain}
                  ).ToPtr());

  //std::vector<frc2::CommandPtr> autoCommands;
  frc2::Command* currentAuto = NoAutoCommand.get();

};
