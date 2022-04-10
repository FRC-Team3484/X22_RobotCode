#ifndef SC_DifferentialDrive_h
#define SC_DifferentialDrive_h

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>

#include "FRC3484_Lib/components/SC_PID.h"

#include "units/length.h"
#include "units/velocity.h"
#include "units/angular_acceleration.h"
 
namespace SC
{ 
    class SC_DifferentialDrive
    {
        public: 
            SC_DifferentialDrive(units::inch_t trackwidth); 
            SC_DifferentialDrive(units::inch_t trackwidth,
                                 units::velocity::feet_per_second_t MaxlinearVel,
                                 units::angular_velocity::degrees_per_second_t MaxAngularVel);
            
            ~SC_DifferentialDrive();

            void Drive_Vel( double VelSP,     //feet/sec
                            double zRotation, //deg/sec
                            double VelPV,     //feet/sec
                            bool DirectDrive);

            void Drive_Pos();

            void SetPIDParams(SC_PIDConstants pidc);

            double GetWheelOutput(SC_Wheel side);

        private: 
            frc::DifferentialDriveWheelSpeeds _DDIK(double tangVel, double zRot); // Diff. Drive Iverse Kinematics

            double maxLinearSpeed;
            double maxRotationSpeed;

            frc::ChassisSpeeds csInput, csPV; //chassis speed
            frc::DifferentialDriveWheelSpeeds wsInput, wsPV; //wheel speed

            frc::DifferentialDriveKinematics *ddKinematics;
            //frc::DifferentialDriveOdometry *ddOdometry; //where is the robot and what way is it facing

            //PID Loops
            SC_PID *pid;
    };
}

#endif 