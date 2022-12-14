//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

//FRC Includes
#include <wpi/array.h>

//Team 302 Includes
#include <chassis/swerve/SwerveChassis.h>

#include <states/chassis/RobotDrive.h>
#include <states/chassis/FieldDrive.h>
#include <states/chassis/HoldDrive.h>
#include <states/chassis/StopDrive.h>
#include <states/chassis/TrajectoryDrive.h>

#include <states/chassis/orientation/MaintainHeading.h>
#include <states/chassis/orientation/SpecifiedHeading.h>
#include <states/chassis/orientation/FaceGoalHeading.h>

/// DEBUG
#include <utils/Logger.h>`

SwerveChassis::SwerveChassis
(
    std::shared_ptr<SwerveModule>                               frontLeft, 
	std::shared_ptr<SwerveModule>                               frontRight,
	std::shared_ptr<SwerveModule>                               backLeft, 
	std::shared_ptr<SwerveModule>                               backRight, 
    units::length::inch_t                                       wheelDiameter,
	units::length::inch_t                                       wheelBase,
	units::length::inch_t                                       track,
	units::velocity::meters_per_second_t                        maxSpeed,
	units::radians_per_second_t                                 maxAngularSpeed,
	units::acceleration::meters_per_second_squared_t            maxAcceleration,
	units::angular_acceleration::radians_per_second_squared_t   maxAngularAcceleration
) : m_frontLeft(frontLeft), 
    m_frontRight(frontRight), 
    m_backLeft(backLeft), 
    m_backRight(backRight), 
    m_wheelDiameter(wheelDiameter),
    m_wheelBase(wheelBase),
    m_track(track),
    m_maxSpeed(maxSpeed),
    m_maxAngularSpeed(maxAngularSpeed), 
    m_maxAcceleration(maxAcceleration), //Not used at the moment
    m_maxAngularAcceleration(maxAngularAcceleration), //Not used at the moment
    m_swerveDriveStates(),
    m_swerveOrientation()
{
    m_odometry = new SwerveOdometry(m_frontLeft, m_frontRight, m_backLeft, m_backRight, m_track, m_wheelBase);

    m_frontLeft->ZeroAlignModule();
    m_frontRight->ZeroAlignModule();
    m_backLeft->ZeroAlignModule();
    m_backRight->ZeroAlignModule();

    //Create map of all swerve drive orientation types
    m_swerveOrientation[SwerveEnums::HeadingOption::MAINTAIN] = new MaintainHeading(SwerveEnums::HeadingOption::MAINTAIN);
    m_swerveOrientation[SwerveEnums::HeadingOption::SPECIFIED_ANGLE] = new SpecifiedHeading(SwerveEnums::HeadingOption::SPECIFIED_ANGLE, units::angle::degree_t(0.0));
    m_swerveOrientation[SwerveEnums::HeadingOption::TOWARD_GOAL] = new FaceGoalHeading(SwerveEnums::HeadingOption::TOWARD_GOAL);

    m_currentOrientation = m_swerveOrientation[SwerveEnums::MAINTAIN];

    //Create map of all swerve drive state types
    m_swerveDriveStates[SwerveEnums::SwerveDriveStateType::ROBOT_DRIVE] =  new RobotDrive(
                                                            SwerveEnums::SwerveDriveStateType::ROBOT_DRIVE, 
                                                            ChassisMovement{}, 
                                                            m_swerveOrientation[SwerveEnums::MAINTAIN]);
    m_swerveDriveStates[SwerveEnums::SwerveDriveStateType::FIELD_DRIVE] = new FieldDrive(*dynamic_cast<RobotDrive*>(m_swerveDriveStates[SwerveEnums::SwerveDriveStateType::ROBOT_DRIVE]));
    m_swerveDriveStates[SwerveEnums::SwerveDriveStateType::HOLD_DRIVE] = new HoldDrive(
                                                            SwerveEnums::SwerveDriveStateType::HOLD_DRIVE, 
                                                            ChassisMovement{}, 
                                                            m_swerveOrientation[SwerveEnums::MAINTAIN]);
    m_swerveDriveStates[SwerveEnums::SwerveDriveStateType::STOP_DRIVE] = new StopDrive(
                                                            SwerveEnums::SwerveDriveStateType::STOP_DRIVE, 
                                                            ChassisMovement{}, 
                                                            m_swerveOrientation[SwerveEnums::MAINTAIN]);
    m_swerveDriveStates[SwerveEnums::SwerveDriveStateType::TRAJECTORY_DRIVE] = new TrajectoryDrive(*dynamic_cast<RobotDrive*>(m_swerveDriveStates[SwerveEnums::SwerveDriveStateType::ROBOT_DRIVE]));
}

void SwerveChassis::SetEncodersToZero()
{
    m_frontLeft.get()->SetEncodersToZero();
    m_frontRight.get()->SetEncodersToZero();
    m_backLeft.get()->SetEncodersToZero();
    m_backRight.get()->SetEncodersToZero();
}

void SwerveChassis::ZeroAlignSwerveModules()
{
    m_frontLeft->ZeroAlignModule();
    m_frontRight->ZeroAlignModule();
    m_backLeft->ZeroAlignModule();
    m_backRight->ZeroAlignModule();
}

void SwerveChassis::Drive()
{
    auto states = wpi::array(m_currentDriveState->CalcSwerveModuleStates());

    //auto kinematics = m_odometry->GetSwerveDriveKinematics();
    //kinematics.DesaturateWheelSpeeds(&states, m_maxSpeed);

    auto [fl,fr,bl,br] = states;

    m_frontLeft.get()->SetDesiredState(fl);
    m_frontRight.get()->SetDesiredState(fr);
    m_backLeft.get()->SetDesiredState(bl);
    m_backRight.get()->SetDesiredState(br);

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("Swerve Chassis"), std::string("BLState Tgt Angle"), bl.angle.Degrees().to<double>() );
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("Swerve Chassis"), std::string("BRState Tgt Angle"), br.angle.Degrees().to<double>() );
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("Swerve Chassis"), std::string("FLState Tgt Angle"), fl.angle.Degrees().to<double>() );
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("Swerve Chassis"), std::string("FRState Tgt Angle"), fr.angle.Degrees().to<double>() );
    
}

void SwerveChassis::Drive(SwerveDriveState* targetState)
{
    m_currentDriveState = targetState;
    m_currentOrientation = targetState->GetDriveOrientation();

    //m_currentDriveState->Init();
    
    Drive();
}

//Seems redundant to have method like this (getpose, reset pose) when we cana access odometry with GetOdometry
frc::Pose2d SwerveChassis::GetPose()
{
    return m_odometry->GetPose();
}

void SwerveChassis::ResetPose(frc::Pose2d pose)
{
    m_odometry->ResetPose(pose);
}

void SwerveChassis::UpdateOdometry()
{
    m_odometry->UpdateOdometry();
}

ISwerveDriveOrientation* SwerveChassis::GetOrientation(SwerveEnums::HeadingOption orientationOption)
{
    return m_swerveOrientation[orientationOption];
}

SwerveDriveState* SwerveChassis::GetDriveState(SwerveEnums::SwerveDriveStateType stateType)
{
    return m_swerveDriveStates[stateType];
}