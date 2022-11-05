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

//Team 302 Includes
#include <chassis/swerve/SwerveChassis.h>

#include <states/chassis/RobotDrive.h>
#include <states/chassis/FieldDrive.h>
#include <states/chassis/HoldDrive.h>
#include <states/chassis/StopDrive.h>
#include <states/chassis/TrajectoryDrive.h>

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
    m_odometry(new SwerveOdometry())
{
    m_frontLeft->ZeroAlignModule();
    m_frontRight->ZeroAlignModule();
    m_backLeft->ZeroAlignModule();
    m_backRight->ZeroAlignModule();

    //Is this the best way to do this?
    m_swerveOrientation[SwerveEnums::MAINTAIN] = new ISwerveDriveOrientation(SwerveEnums::HeadingOption::MAINTAIN);
    //..... continue

    m_swerveDriveStates[SwerveDriveState::RobotDrive] =  new RobotDrive(SwerveDriveState::RobotDrive, 
                                                            ChassisMovement{}, 
                                                            *m_swerveOrientation[SwerveEnums::MAINTAIN]);

    //...... continue doing this, waiting to finish until I determine if this is right idea
}

void SwerveChassis::Drive()
{
    auto states = wpi::array(m_currentDriveState->CalcSwerveModuleStates());

    auto kinematics = m_odometry->GetSwerveKinematics();
    kinematics.DesaturateWheelSpeeds(&states, m_maxSpeed);

    auto [fl,fr,bl,br] = states;

    m_frontLeft.get()->SetDesiredState(fl);
    m_frontRight.get()->SetDesiredState(fr);
    m_backLeft.get()->SetDesiredState(bl);
    m_backRight.get()->SetDesiredState(br);
}