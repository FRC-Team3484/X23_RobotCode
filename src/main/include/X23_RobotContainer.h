#pragma once
#include <tuple>
#include <memory>
#include <map>
#include <unordered_map>
#include <array>
#include <iostream>
#include "FRC3484_Lib/utils/SC_Functions.h"
#include <units/acceleration.h>
#include <units/velocity.h>
#include "Robot.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/PrintCommand.h>

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandPS4Controller.h>

#include <pathplanner/lib/auto/MecanumAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include "constants.h"
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
class Cmd_Elev_HybridZone;
class RobotContainer {
 public:
  RobotContainer();

  void setAutoDefaults();
  void setTeleopDefaults();

  void startAutoCommand();
  void endAutoCommand();

 private: 
  void ConfigureBindings(); 

  /**************
   * Subsystems *
   **************/
  // X23_Drivetrain *_drivetrain = nullptr;
  // X23_Intake *_intake = nullptr;
  // X23_Elevator *_elevator = nullptr;  

  X23_Drivetrain _drivetrain{ std::make_tuple<int, int>(C_FX_FL_MASTER, C_FX_FL_SLAVE),
                              std::make_tuple<int, int>(C_FX_FR_MASTER, C_FX_FR_SLAVE),
								              std::make_tuple<int, int>(C_FX_BL_MASTER, C_FX_BL_SLAVE),
                              std::make_tuple<int, int>(C_FX_BR_MASTER, C_FX_BR_SLAVE),
                              SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::CTREPCM, C_DRIVE_SOL}, C_PIGEON_IMU};
  X23_Intake _intake{C_SPX_INTAKE_LEFT, C_SPX_INTAKE_RIGHT};
  X23_Elevator _elevator{ C_FX_ELEVATEMOTOR,
                          C_FX_TILTMOTOR,
                          SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::CTREPCM, C_SOL_CLAW_GRIP},
                          SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::CTREPCM, C_SOL_CLAW_TILT},
                          SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::CTREPCM, C_SOL_ELEVATOR_BRAKE},
                          C_DI_CH_ELEVATOR_TILT_HOME,
                          C_DI_CH_ELEVATOR_HOME, 
                          C_DI_CH_ELEVATOR_TILT_MAX};

  /****************
   * Path Planner *
   ****************/

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> FunEvents {
    // Drivetrain Commands
    //{"drivetrain_balance", std::make_shared<BalanceCommand>()},

    //Manipulator Commands
    {"manipulator_compact", std::make_shared<Cmd_Elev_HybridZone>(&_elevator)}/*,

    {"manipulator_cube_high", manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::cubeHighState
    ).Unwrap()},

    {"manipulator_cube_mid", manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::cubeMidState
    ).Unwrap()},

    {"manipulator_cube_pickup", manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::cubePickupState
    ).Unwrap()},

    {"manipulator_cone_mid", manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::coneMidState
    ).Unwrap()},

    {"manipulator_cone_pickup", manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::conePickupState
    ).Unwrap()},

    {"manipulator_substation", manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::substationState
    ).Unwrap()},
 
    // Claw Commands
    {"claw_close", clawSubsystem.getClosedCommand(true).Unwrap()},
    {"claw_open", clawSubsystem.getClosedCommand(false).Unwrap()},*/
  };

  // Auto Builder
  pathplanner::MecanumAutoBuilder autoBuilder 
  {
    [this]() { return _drivetrain.GetPose(); }, // Function to supply current robot pose
    [this](auto initPose) { _drivetrain.SetPose(initPose); }, // Function used to reset odometry at the beginning of auto
    pathplanner::PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    pathplanner::PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    [this](auto speeds) { _drivetrain.DriveAuto(speeds); }, // Output function that accepts field relative ChassisSpeeds
    FunEvents, 
    {&_drivetrain}, 
    false
  };

  // List of possible autos and relevant configs.
  std::array<std::string, 10> autoNames {
    {
      "left",
      "leftBalance",
      "leftNoPlatform",
      "middleBalance",
      "middleNoPlatform",
      "Right",
      "RightBalance",
      "RightNoPlatform",
    }
  };

  /****************
   * Auto Chooser *
   ****************/
  frc2::CommandPtr noAutoCommand = frc2::PrintCommand("NO AUTO\n").ToPtr();
  std::vector<frc2::CommandPtr> autoCommands;
  frc2::Command* currentAuto = noAutoCommand.get();

  frc::SendableChooser<frc2::Command*> autonomousChooser;
};