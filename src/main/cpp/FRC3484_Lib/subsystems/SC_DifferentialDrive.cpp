#include "FRC3484_Lib/subsystems/SC_DifferentialDrive.h"

#include "units/math.h"

using namespace SC;
using namespace frc;
using namespace units::length;
using namespace units::angle;
using namespace units::velocity;
using namespace units::angular_velocity;

//Constructers >>
SC_DifferentialDrive::SC_DifferentialDrive(inch_t trackWidth)
{
    pid = new SC_PID();
    
    if(pid != NULL)
    {
        pid->Enable();
        pid->DisableManualMode();
    }
    
    ddKinematics = new frc::DifferentialDriveKinematics(trackWidth);
    //ddOdometry = new frc::DifferentialDriveOdometry();

    maxLinearSpeed = 1.0; //ft/s
    maxRotationSpeed = 1.0;
}

SC_DifferentialDrive::SC_DifferentialDrive(inch_t trackWidth, feet_per_second_t MaxLinearVel, degrees_per_second_t MaxAngularVel)
{
    pid = new SC_PID();

    if(pid != NULL)
    {
        pid->Enable();
        pid->DisableManualMode();
    }

     ddKinematics = new frc::DifferentialDriveKinematics(trackWidth);
    //ddOdometry = new frc::DifferentialDriveOdometry();

    maxLinearSpeed = units::math::abs(MaxLinearVel).value();
    maxRotationSpeed = units::math::abs(MaxAngularVel).value();

    wsInput = DifferentialDriveWheelSpeeds{0_mps, 0_mps};
}

//deconstructor >>
SC_DifferentialDrive::~SC_DifferentialDrive()
{
    if(pid != NULL) delete pid;
    if(ddKinematics != NULL) delete pid;
}

void SC_DifferentialDrive::Drive_Vel(double VelSP, double zRotation, double VelPV, bool DirectDrive) 
{
    wsInput = this->_DDIK(VelSP, zRotation);

    if(pid!=NULL)
    {

        if(DirectDrive) //was directdrive
        {
            pid->SetSP(F_Scale(-maxLinearSpeed, maxLinearSpeed, 0.0, 100.0, VelSP));
            pid->Calculate(F_Scale(-maxLinearSpeed, maxLinearSpeed, 0.0, 100.0, VelSP));
        }
    }
}

void SC_DifferentialDrive::Drive_Pos()
{

}

void SC_DifferentialDrive::SetPIDParams(SC_PIDConstants pidc) 
{
    if((pid!=NULL)) { pid->SetPIDConstants(pidc);}
}
//PID loop constants ^


double SC_DifferentialDrive::GetWheelOutput(SC_Wheel side)
{
    switch (side)
    {
    case LEFT_WHEEL:
        return F_Scale(-maxLinearSpeed, maxLinearSpeed,
                        -1.0, 1.0,
                        wsInput.left.value());
    case RIGHT_WHEEL:
        return F_Scale(-maxLinearSpeed, maxLinearSpeed,
                        -1.0, 1.0,
                        wsInput.right.value());
    default:
        return 0;
    }
}

DifferentialDriveWheelSpeeds SC_DifferentialDrive::_DDIK(double tangVel, double zRot) //zRot (zRotation = zRot)
{

    double leftSpeed, rightSpeed;

    double maxInput =
      std::copysign(std::max(std::abs(tangVel), std::abs(zRot)), tangVel);

    if (tangVel >= 0.0) 
    {
        // First quadrant, else second quadrant
         if (zRot >= 0.0) 
        {
            leftSpeed = maxInput;
            rightSpeed = tangVel - zRot;
        } 
        else 
        {
            leftSpeed = tangVel + zRot;
            rightSpeed = maxInput;
        }
    } 
    else 
    {
        // Third quadrant, else fourth quadrant
         if (zRot >= 0.0) 
         {
            leftSpeed = tangVel + zRot;
            rightSpeed = maxInput;
        }   
        else 
        {
            leftSpeed = maxInput;
            rightSpeed = tangVel - zRot;
        }
    }

    // Normalize the wheel speeds
    double maxMagnitude = std::max(std::abs(leftSpeed), std::abs(rightSpeed));
    if (maxMagnitude > 1.0) 
    {
        leftSpeed /= maxMagnitude;
        rightSpeed /= maxMagnitude;
    }

    return {units::make_unit<feet_per_second_t>(leftSpeed),
            units::make_unit<feet_per_second_t>(rightSpeed)};
}