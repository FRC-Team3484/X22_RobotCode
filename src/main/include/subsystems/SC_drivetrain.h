#ifndef SC_drivetrain_h
#define SC_drivetrain_h

// #include "FRC3484_Lib/subsystems/SC_DifferentialDrive.h"

#include "Constants.h"
#include "Globals.h"

#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "FRC3484_Lib/utils/SC_Shuffleboard.h"

#include "units/length.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"

#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"

#include "frc/drive/DifferentialDrive.h"
#include "frc/kinematics/DifferentialDriveWheelSpeeds.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/kinematics/DifferentialDriveOdometry.h"

#include "frc/Solenoid.h"
//#include "frc/DoubleSolenoid.h"

#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableValue.h"

class Drivetrain
{
public:
	Drivetrain();
	Drivetrain(units::length::inch_t trackwidth,
				units::velocity::feet_per_second_t MaxVel,
				units::angular_velocity::degrees_per_second_t MaxRotVel,
				std::tuple<int, int> LeftIDs, std::tuple<int, int> RightIDs,
				SC::SC_Solenoid shifter);


	~Drivetrain();

	void Drive(double Throttle, double Rotation, bool ShiftOverride, bool EBrake);

	// Shifts to high gear if `override` is false.
	void Shift(bool shiftLow, bool shiftHigh);
	
	void SetCoastMode();
	void SetBrakeMode();

private:
	double _CalcWheelVelocity(double counts);

	void _UpdateDashboard();
	void _InitDashboard();
	void _InitMotor(ctre::phoenix::motorcontrol::can::WPI_TalonFX* Motor, bool Invert, ctre::phoenix::motorcontrol::can::WPI_TalonFX* Master = NULL);

	//SC::SC_DifferentialDrive *drive;
	frc::DifferentialDrive              *drive;
	frc::DifferentialDriveKinematics    *ddKinematics;
	frc::DifferentialDriveOdometry		*ddOdometry;
	frc::DifferentialDrive::WheelSpeeds wsInput;
	frc::DifferentialDriveWheelSpeeds   ws_PV, ws_PVf;//, ws_chassis;
	frc::ChassisSpeeds                  cs_PV;

	ctre::phoenix::motorcontrol::can::WPI_TalonFX *Motor_Left_Master, *Motor_Left_Slave;
	ctre::phoenix::motorcontrol::can::WPI_TalonFX *Motor_Right_Master, *Motor_Right_Slave;

	double throttleDemand, rotationDemand, throttleCoeff;
	bool inHighGear, inLowGear;

	frc::Solenoid *_shifter;

	SC::SC_Range<double> linVelRange;
	SC::SC_Range<double> angVelRange;  

	SC::SC_ABFilterU<units::velocity::meters_per_second_t> *_ws_filter_left, *_ws_filter_right;

	// Dashboard
	nt::NetworkTableInstance _nt_inst;
	std::shared_ptr<nt::NetworkTable> _nt_table;
};


#endif // SC_drivetrain_H (for Completeness)
