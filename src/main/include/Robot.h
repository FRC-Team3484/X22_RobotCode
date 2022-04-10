// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "FRC3484_Lib/utils/SC_Datatypes.h"

#include "RobotContainer.h"

#include "subsystems/SC_drivetrain.h"
#include "subsystems/SC_Intake.h"
#include "subsystems/SC_Launcher.h"
#include "subsystems/SC_Climb.h"

#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/Debouncer.h>
#include <frc/PneumaticHub.h>
#include <frc/PowerDistribution.h>

#include <frc/Timer.h>


#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

class Robot : public frc::TimedRobot 
{
public:
	void RobotInit() override;
	void RobotPeriodic() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

private:
	// Have it null by default so that if testing teleop it
	// doesn't have undefined behavior and potentially crash.
	frc2::Command* m_autonomousCommand = nullptr;

	// Processes the driver inputs for moving the robots. Returns true if the robot should shift to low gear
	bool _HandleDriverInput();
	void _HandleGameDeviceInput();
#ifdef CLIMB_CONTROL_SEPARATE
	void _HandleClimbInput();
#endif

	RobotContainer m_container;

	std::shared_ptr<nt::NetworkTable> _nt_table;

	frc::PneumaticHub *pch;
	frc::PowerDistribution *pdp;

	Drivetrain *_drivetrain;
	SC_Intake *_intake;
	SC_Launcher *_launcher;
	SC_Climb *_climb;

	SC::SC_Range<double> Throttle_Range_Normal;
	SC::SC_Range<double> Throttle_Range_Fine;

#if defined(DRIVE_MODE_ARCADE) || defined(DRIVE_MODE_ARCADE)
	double throttleDemand, turnDemand;
#elif defined(DRIVE_MODE_TANK)
	double rightDemand, leftDemand;
#endif
	bool forceLowGear;

	frc::XboxController *GP1_Driver; // GP = Gamepad
	frc::XboxController *BB_GameDevice;
#ifndef CLIMB_CONTROL_SPERATE
	frc::Joystick       *JS_Climb; 
#endif

	// Autonomous
	frc::Debouncer *_dbnc_taxi_time;
	frc::Timer *_tmr_taxi_time;
	bool _launch, _autoDone;
};