#include "subsystems/X22_Drivetrain.h"

#include "units/math.h"

using namespace SC;
using namespace frc;
using namespace nt;
using namespace units::length;
using namespace units::velocity;
using namespace units::angular_velocity;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;


X22_Drivetrain::X22_Drivetrain(inch_t trackwidth, feet_per_second_t MaxTangVel, degrees_per_second_t MaxRotVel,
                        std::tuple<int, int> LeftIDs, std::tuple<int, int> RightIDs, SC_Solenoid Shifter)
{
	// drive = new SC_DifferentialDrive(trackwidth, MaxTangVel, MaxRotVel);
	this->_InitDashboard(); 

	linVelRange(-MaxTangVel.value(), MaxTangVel.value());
	angVelRange(-MaxRotVel.value(), MaxRotVel.value());

	this->_ws_filter_left = new SC_ABFilterU<meters_per_second_t>(C_DT_WHEEL_TAU, C_SCAN_TIME);
	this->_ws_filter_right = new SC_ABFilterU<meters_per_second_t>(C_DT_WHEEL_TAU, C_SCAN_TIME);

	int sCh = -1;

	// Initialize front right wheel and slave controller
	if(LeftIDs != C_BLANK_IDS) 
	{ 
		Motor_Left_Master = new WPI_TalonFX(std::get<0>(LeftIDs));
		_InitMotor(Motor_Left_Master, true);

		sCh = std::get<1>(LeftIDs);

		if(sCh != C_DISABLED_CHANNEL) { Motor_Left_Slave = new WPI_TalonFX(sCh); _InitMotor(Motor_Left_Slave, true, Motor_Left_Master); }
		else { Motor_Left_Slave = NULL; }
	} 
	else { Motor_Left_Master = NULL; Motor_Left_Slave = NULL; }
	

	// Initialize front left wheel and slave controller
	 if(RightIDs != C_BLANK_IDS) 
	{ 
		Motor_Right_Master = new WPI_TalonFX(std::get<0>(RightIDs));
		_InitMotor(Motor_Right_Master, false);

		sCh = std::get<1>(RightIDs);

		if(sCh != C_DISABLED_CHANNEL) { Motor_Right_Slave = new WPI_TalonFX(sCh); _InitMotor(Motor_Right_Slave, false, Motor_Right_Master); }
		else { Motor_Right_Slave = NULL; }
	} 
	else { Motor_Right_Master = NULL; Motor_Right_Slave = NULL; }
	
	drive = new DifferentialDrive(*Motor_Left_Master, *Motor_Right_Master);
	ddKinematics = new DifferentialDriveKinematics(trackwidth);
	ddOdometry = new DifferentialDriveOdometry(frc::Rotation2d{0_deg}, 0_m, 0_m);

	if(Shifter.Channel != -1) 
	{
		_shifter = new Solenoid(Shifter.CtrlID, Shifter.CtrlType, Shifter.Channel);
		Shift(true, false);
	}
	else { _shifter = NULL; }
}

X22_Drivetrain::~X22_Drivetrain()
{
	if(drive != NULL) { delete drive; }
	if(ddKinematics != NULL) { delete ddKinematics; }
	if(ddOdometry != NULL) { delete ddOdometry; }
	if(Motor_Left_Master != NULL) { delete Motor_Left_Master; }
	if(Motor_Left_Slave != NULL) { delete Motor_Left_Slave; }
	if(Motor_Right_Master != NULL) { delete Motor_Right_Master; }
	if(Motor_Right_Slave != NULL) { delete Motor_Right_Slave; }
	if(_shifter != NULL) { delete _shifter; }
	if(_ws_filter_left != NULL) { delete _ws_filter_left; }
	if(_ws_filter_right != NULL) { delete _ws_filter_right; }

	// if(ntLeftOut != NULL) { delete ntLeftOut; }
	// if(ntRightOut != NULL) { delete ntRightOut; }
	// if(ntThrottleIn != NULL) { delete ntThrottleIn; }
	// if(ntRotationIn != NULL) { delete ntRotationIn; }
	// if(ntInLowGear != NULL) { delete ntInLowGear; }
	// if(ntInHighGear != NULL) { delete ntInHighGear; }
	//if(ntRampTime != NULL) { delete ntRampTime; }
	// if(ntLeftVel != NULL) { delete ntLeftVel; }
	// if(ntRightVel != NULL) { delete ntRightVel; }
	// if(ntChassisVx != NULL) { delete ntChassisVx; }
	// if(ntChassisVy != NULL) { delete ntChassisVy; }
	// if(ntChassisOmega != NULL) { delete ntChassisOmega; }
}

void X22_Drivetrain::Drive(double Throttle, double Rotation, bool ShiftOverride, bool EBrake)
{
	this->throttleCoeff = this->_nt_table->GetNumber("Throttle Scale Coeff", C_THROTTLE_SCALE_COEFF);

	this->throttleDemand = std::copysign(std::min((Throttle*Throttle)*this->throttleCoeff, std::abs(Throttle)), Throttle);
	this->rotationDemand = Rotation;
	
	if(Motor_Left_Master != NULL) 	{ Motor_Left_Master->ConfigOpenloopRamp(0.5); }//this->_nt_table->GetNumber("DT Ramp Time", 2.0)); }
	if(Motor_Right_Master != NULL) 	{ Motor_Right_Master->ConfigOpenloopRamp(0.5); }//this->_nt_table->GetNumber("DT Ramp Time", 2.0)); }
	if(Motor_Left_Slave != NULL) 	{ Motor_Left_Slave->ConfigOpenloopRamp(0.5); }//this->_nt_table->GetNumber("DT Ramp Time", 2.0)); }
	if(Motor_Right_Slave != NULL) 	{ Motor_Right_Slave->ConfigOpenloopRamp(0.5); }//this->_nt_table->GetNumber("DT Ramp Time", 2.0)); }

	if((drive != NULL) && (Motor_Left_Master != NULL) && (Motor_Right_Master != NULL))
	{
	   	// Convert driver inputs to drivetrain outputs
		wsInput = drive->ArcadeDriveIK(this->throttleDemand , -Rotation, false);

		ws_PV.left = units::make_unit<feet_per_second_t>(_CalcWheelVelocity(Motor_Left_Master->GetSelectedSensorVelocity(0)));
		ws_PV.right = units::make_unit<feet_per_second_t>(_CalcWheelVelocity(Motor_Right_Master->GetSelectedSensorVelocity(0)));

		if((_ws_filter_left != NULL) && (_ws_filter_right != NULL) && (ddKinematics != NULL))
		{
			// Apply Alpha-Beta filter to smooth out encoder feedback
			ws_PVf.left = this->_ws_filter_left->Filter(ws_PV.left);
			ws_PVf.right = this->_ws_filter_right->Filter(ws_PV.right);

			cs_PV = ddKinematics->ToChassisSpeeds(ws_PVf);
		}
		else { cs_PV.vx = ws_PV.left; cs_PV.vy = 0.0_mps; cs_PV.omega = 0.0_rad_per_s; }

		// Auto-shifting logic
		Shift(ShiftOverride || (units::math::abs(cs_PV.vx) < C_SHIFT_DOWN_SPEED),
			 !ShiftOverride && (units::math::abs(cs_PV.vx) > C_SHIFT_UP_SPEED));

		// Set motor outputs (direct control)

		if(EBrake)
		{
			SetBrakeMode();
			Motor_Left_Master->Set(ControlMode::PercentOutput, 0.0);
			Motor_Right_Master->Set(ControlMode::PercentOutput, 0.0);
		}
		else
		{
			SetCoastMode();
			Motor_Left_Master->Set(ControlMode::PercentOutput, wsInput.left);
			Motor_Right_Master->Set(ControlMode::PercentOutput, wsInput.right);
		}

		// Send subsystem data to the dashboard
		this->_UpdateDashboard();

		// Handle motor safety
		drive->Feed();
		drive->FeedWatchdog();
		drive->Check();
		drive->CheckMotors();	
	}
}

void X22_Drivetrain::Shift(bool shiftLow, bool shiftHigh)
{
	if(_shifter != NULL)
	{
		_shifter->Set(shiftHigh && !shiftLow);
	}

	inLowGear = shiftLow;
	inHighGear = shiftHigh;
}

double X22_Drivetrain::_CalcWheelVelocity(double counts)
{
	if(this->inLowGear) { return counts * C_DT_SCALE_FACTOR_LO; }
	else if (this->inHighGear) { return counts * C_DT_SCALE_FACTOR_HI; }	
	else{ return 0; }
}

void X22_Drivetrain::_UpdateDashboard()
{
	this->_nt_table->PutNumber("Left Velocity", units::convert<meters_per_second, feet_per_second>(ws_PVf.left).value());
	this->_nt_table->PutNumber("Right Velocity", units::convert<meters_per_second, feet_per_second>(ws_PVf.right).value());

	this->_nt_table->PutBoolean("In High Gear", this->inHighGear);
	this->_nt_table->PutBoolean("In Low Gear" , this->inLowGear);

	// if(ntThrottleIn != NULL) { ntThrottleIn->SetNumber(this->throttleDemand); }
	// if(ntRotationIn != NULL) { ntRotationIn->SetNumber(this->rotationDemand); }
	
	// if(ntLeftOut != NULL) { ntLeftOut->SetNumber(this->wsInput.left); }
	// if(ntRightOut != NULL) { ntRightOut->SetNumber(this->wsInput.right); }

	// if(ntLeftVel != NULL) { ntLeftVel->SetNumber(units::convert<meters_per_second, feet_per_second>(ws_PVf.left).value()); }
	// if(ntRightVel != NULL) { ntRightVel->SetNumber(units::convert<meters_per_second, feet_per_second>(ws_PVf.right).value()); }

	// if(ntChassisVx != NULL) { ntChassisVx->SetNumber(units::convert<meters_per_second, feet_per_second>(cs_PV.vx).value()); }
	// if(ntChassisVy != NULL) { ntChassisVy->SetNumber(units::convert<meters_per_second, feet_per_second>(cs_PV.vy).value()); }
	// if(ntChassisOmega != NULL) { ntChassisOmega->SetNumber(units::convert<radians_per_second, degrees_per_second>(cs_PV.omega).value()); }

	// if(ntInLowGear != NULL) { ntInLowGear->SetBool(this->inLowGear); }
	// if(ntInHighGear != NULL) { ntInHighGear->SetBool(this->ntInHighGear); }

	// if(ntLeftCurrent != NULL) { ntLeftCurrent->SetNumber(Motor_Left_Master->GetOutputCurrent()); }
	// if(ntRightCurrent != NULL) { ntRightCurrent->SetNumber(Motor_Right_Master->GetOutputCurrent()); }
}

void X22_Drivetrain::_InitDashboard()
{
    this->_nt_inst = NetworkTableInstance::GetDefault();
    this->_nt_table = this->_nt_inst.GetTable("X22");

	this->_nt_table->PutNumber("DT Ramp Time", 2.0);
	this->_nt_table->PutNumber("Throttle Scale Coeff", C_THROTTLE_SCALE_COEFF);

	this->_nt_table->PutNumber("Left Velocity", 0.0);
	this->_nt_table->PutNumber("Right Velocity", 0.0);
	this->_nt_table->PutBoolean("In High Gear", false);
	this->_nt_table->PutBoolean("In Low Gear", true);
	
	/*
	ntLeftOut = DebugDash->AddNumBar("Left Output", props_numbar);
	ntRightOut = DebugDash->AddNumBar("Right Output", props_numbar);
	ntLeftVel = DebugDash->AddNumBar("Left Velocity", props_diffdrive_hi);
	ntRightVel = DebugDash->AddNumBar("Right Velocity", props_diffdrive_hi);
	ntChassisVx = DebugDash->AddNumBar("Chassis Vx", props_diffdrive_hi);
	ntChassisVy = DebugDash->AddNumBar("Chassis Vy", props_diffdrive_hi);
	ntChassisOmega = DebugDash->AddNumBar("Chassis ω", props_diffdrive_hi);
	ntThrottleIn = DebugDash->AddNumBar("Throttle In", props_numbar);
	ntRotationIn = DebugDash->AddNumBar("Rotation In", props_numbar);
	ntLeftCurrent = DebugDash->AddNumBar("I_Left", props_current);
	ntRightCurrent = DebugDash->AddNumBar("I_Right", props_current);
	*/

	//ntRampTime = DebugDash->AddEntry<double>("DT Ramp Time", 2.0);
	//ntThrottleScale = DebugDash->AddEntry<double>("Throttle Scale Coeff", C_THROTTLE_SCALE_COEFF);
}

void X22_Drivetrain::_InitMotor(WPI_TalonFX* Motor, bool Invert, WPI_TalonFX* Master)
{
	if(Motor != NULL)
	{
		Motor->SetInverted(Invert);
		Motor->SetNeutralMode(NeutralMode::Brake);
		Motor->ConfigOpenloopRamp(2);
		Motor->ConfigClosedloopRamp(0);
		Motor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
		Motor->SetSelectedSensorPosition(0);

		if(Master != NULL) { Motor->Follow(*Master); }
	}
}

void X22_Drivetrain::SetBrakeMode()
{
	if(this->Motor_Left_Master != NULL) { this->Motor_Left_Master->SetNeutralMode(NeutralMode::Brake); }
	if(this->Motor_Left_Slave != NULL) { this->Motor_Left_Slave->SetNeutralMode(NeutralMode::Brake); }
	if(this->Motor_Right_Master != NULL) { this->Motor_Right_Master->SetNeutralMode(NeutralMode::Brake); }
	if(this->Motor_Right_Slave != NULL) { this->Motor_Right_Slave->SetNeutralMode(NeutralMode::Brake); }
}

void X22_Drivetrain::SetCoastMode()
{
	if(this->Motor_Left_Master != NULL) { this->Motor_Left_Master->SetNeutralMode(NeutralMode::Coast); }
	if(this->Motor_Left_Slave != NULL) { this->Motor_Left_Slave->SetNeutralMode(NeutralMode::Coast); }
	if(this->Motor_Right_Master != NULL) { this->Motor_Right_Master->SetNeutralMode(NeutralMode::Coast); }
	if(this->Motor_Right_Slave != NULL) { this->Motor_Right_Slave->SetNeutralMode(NeutralMode::Coast); }
}