// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "units/velocity.h"
#include "units/length.h"
#include "units/constants.h"

#include "FRC3484_Lib/utils/SC_ControllerMaps.h"

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#define C_DISABLED_CHANNEL			-1	// Device or channel is not used.

/*==========*/
/* CAN ID's */
/*==========*/
#define C_FX_FL_MASTER				1
#define C_FX_FL_SLAVE			   -1
#define C_FX_FR_MASTER				2
#define C_FX_FR_SLAVE		   	   -1
#define C_FX_BL_MASTER				3
#define C_FX_BL_SLAVE			   -1
#define C_FX_BR_MASTER				4
#define C_FX_BR_SLAVE			   -1
#define C_SPX_INTAKE                5
#define C_SPX_FEED_MASTER           6
#define C_SPX_FEED_SLAVE            7
#define C_PIGEON_IMU			   20
#define C_PCM					   30
#define C_PDP					   40


/*===================*/
/* Solenoid Channels */
/*===================*/
#define C_DRIVE_SOL					0 // Shifter
#define C_SOL_INTAKE				1

/*==============*/
/* DIO Channels */
/*==============*/
#define C_DI_CH_STORED_SW			0
#define C_DI_CH_TURRET_LS_MIN		1
#define C_DI_CH_TURRET_LS_MAX		2
#define C_DI_CH_LOADER_DOWN_SW		3


/*==========*/
/* Settings */
/*==========*/
#define C_DRIVE_DEADBAND			0.05    // 5% Joystick input
#define C_DRIVE_MAX_DEMAND			0.95    // Joystick input scale range (+/-) for normal movements
#define C_DRIVE_MAX_DEMAND_FINE		0.5     // Joystick input scale range (+/-) for fine movements
#define C_DT_WHEEL_TAU				20_ms   // Filter time for encoder feedback
#define C_THROTTLE_SCALE_COEFF		1.5     // Scaling Coefficient for throttle input


/*=======================*/
/* Drivetrain Parameters */
/*=======================*/
#define C_X23_TRACK_WIDTH			30.0_in
#define C_X23_DT_WHEEL_DIAM			6.0 // in

#define C_DT_RPM_TO_FPS				(units::constants::pi * C_X23_DT_WHEEL_DIAM) / (60.0 * 12.0)

const double C_GEAR_RATIO		= 1.0 / 18.0;

const double C_DT_MOTOR_MAX_RPM		= 6380.0;
const double C_DT_MOTOR_MAX_RPM_ACT = 6000.0; // TODO: Get max achievable RPM of drivetrain motors.
const double C_DT_ENC_CPR			= 2048.0;

const double C_MAX_GEAR_ENC      = (C_DT_MOTOR_MAX_RPM / 600.0) * (C_DT_ENC_CPR / C_GEAR_RATIO);

const double C_DT_SCALE_FACTOR   = ((600.0 * C_GEAR_RATIO) / C_DT_ENC_CPR) * C_DT_RPM_TO_FPS;

const units::feet_per_second_t C_GEAR_MAX_SPEED 	= 17.0_fps;
const units::feet_per_second_t C_SHIFT_UP_SPEED     = 5.0_fps;
const units::feet_per_second_t C_SHIFT_DOWN_SPEED 	= 3.5_fps;


/*===================*/
/* Intake Parameters */
/*===================*/
#define C_INTAKE_DRIVE_SPEED		1.0 // *100%
#define C_INTAKE_RAMP_TIME			0.50 // Seconds
#define C_FEED_DRIVE_SPEED			1.00 // *100%


/*======================*/
/* CONTROLLER CONSTANTS */
/*======================*/
#define C_DRIVER_USB                 0
#define C_GAMEDEV_USB                1
#define C_BUTTONBOX
// Game Device control input scheme 
// #define GD_SCHEME_JOYSTICK	/* Logitech Extreme-3D Pro Joystick Scheme */
#define GD_SCHEME_XBOX			/* Xbox Controller Scheme */
//#define GD_SCHEME_DS4 		/* DualShock 4 Controller Scheme */



/**
 * Set The Driver mode
 */
//#define DRIVE_MODE_TANK
#define DRIVE_MODE_ARCADE
//#define DRIVE_MODE_CURVE

#if defined(DRIVE_MODE_TANK)
	#define C_DRIVER_LEFT_AXIS			XBOX_LS_Y
	#define C_DRIVER_RIGHT_AXIS			XBOX_RS_Y
#elif defined(DRIVE_MODE_ARCADE)
	#define C_DRIVER_THROTTLE_AXIS		XBOX_LS_Y 
	#define C_DRIVER_STEER_AXIS			XBOX_RS_X //XBOX_LS_X (arcade)
#elif defined(DRIVE_MODE_CURVE)
	#define C_DRIVER_THROTTLE_AXIS		XBOX_LS_Y
	#define C_DRIVER_STEER_AXIS			XBOX_LS_X
#endif

#define C_DRIVER_SHIFT_LOW_BTN        	XBOX_A
#define C_DRIVER_EBRAKE					XBOX_LB

#define DRIVER_FINE_ADJ_MODE
#ifdef DRIVER_FINE_ADJ_MODE
#define C_DRIVE_ADJ_BTN               	XBOX_RB
#endif

/*===================*/
/* Game Device Input */
/*===================*/
#ifdef C_BUTTONBOX
/* intake modes */					
	#define C_SUCK_CUBE					R_BTN_1
	#define C_KNOCK_CONE_L				R_BTN_2
	#define C_KNOCK_CONE_R				R_BTN_3
	#define C_SPIT_CUBES				R_BTN_4
/* elevator modes */					
	#define C_HIGH_CONE					L_BTN_1
	#define C_HIGH_CUBE					L_BTN_2
	#define C_MID_CONE					L_BTN_3
	#define C_MID_CUBE					L_BTN_4
	#define C_LOW						L_BTN_5
	#define C_NEUTRAL					L_BTN_6
	#define C_D_PICKUP					L_BTN_7
/* claw modes */					
	#define C_CLAW_GRAB					R_BTN_5
	#define C_CLAW_PIV					R_BTN_6

#elif defined(GD_SCHEME_JOYSTICK)
	#define C_GD_INTAKE					LE3D_BTN_5
	
#elif defined(GD_SCHEME_DS4)
	#define C_GD_INTAKE					DS4_CROSS
	
#endif

/*==================*/
/* Debouncer Config */
/*==================*/
// Driver inputs
#define C_MANUSHIFT_DBNC_TIME		0.100_s	// Force-low gear shift control input debounce time
#define C_FINEADJ_DBNC_TIME			0.100_s	// Fine-Adjustment controller input debounce time

// Game device inputs
#define C_INTAKE_BTN_DBNC_TIME		0.100_s	// Intake deploy controller input debounce time
#define C_Pincher_BTN_DBNC_TIME     0.100_s // Pincher Debounce Time
// Auto-functions
#define C_AUTOSHIFT_DBNC_TIME		0.250_s	// Auto-shift debounce time

/*=====================*/
/* Action Delay Config */
/*=====================*/
#define C_INTAKEMOTOR_DELAY_TIME	1.0_s	// Time after intake is deployed before the motor starts running
#define C_AUTOFEED_OFF_DELAY_TIME	1.0_s	// Time after `cargo stored` switch is released before stopping the feed motors.

#define C_AUTO_TAXI_TIME			1.0_s

/*===================*/
/* General Constants */
/*===================*/
#define C_SCAN_TIME					0.020_s
#define C_SCAN_TIME_SEC				C_SCAN_TIME.value() // Seconds
const std::tuple<int, int> C_BLANK_IDS = std::make_tuple<int, int>(C_DISABLED_CHANNEL, C_DISABLED_CHANNEL);

/*==================*/
/*PID Loop Variables*/
/*==================*/
#define E_Kp 0.1
#define E_Ki 0.0
#define E_dt 0.01
#define E_Kd 0.1
#define T_Kp 0.1
#define T_Ki 0.1
#define T_dt 0.01
#define T_Kd 0.1