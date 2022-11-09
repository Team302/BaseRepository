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

//C++ Includes
#include <math.h>

//FRC Includes
#include <frc/geometry/Rotation2d.h>
#include <wpi/array.h>
#include <units/length.h>

//Team302 Includes
#include <chassis/swerve/SwerveOdometry.h>
#include <hw/factories/PigeonFactory.h>
#include <utils/Logger.h>
#include <chassis/ChassisFactory.h>

using namespace frc;

SwerveOdometry::SwerveOdometry(
) : m_flPosition(*new SwerveModulePosition()),
    m_frPosition(*new SwerveModulePosition()),
    m_blPosition(*new SwerveModulePosition()),
    m_brPosition(*new SwerveModulePosition())
{
    SwerveChassis* chassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();

    m_frontLeftLocation = frc::Translation2d(chassis->GetWheelBase()/2.0, chassis->GetTrack()/2.0);
    m_frontRightLocation = frc::Translation2d(chassis->GetWheelBase()/2.0, -1.0*chassis->GetTrack()/2.0);
    m_backLeftLocation = frc::Translation2d(-1.0*chassis->GetWheelBase()/2.0, chassis->GetTrack()/2.0);
    m_backRightLocation = frc::Translation2d(-1.0*chassis->GetWheelBase()/2.0, -1.0*chassis->GetTrack()/2.0);

    m_frontLeft = chassis->GetFrontLeft();
    m_frontRight = chassis->GetFrontRight();
    m_backLeft = chassis->GetBackLeft();
    m_backRight = chassis->GetBackRight();

    //default swerve module position
    SwerveModulePosition defaultSwervePose = {units::length::meter_t(0.0), frc::Rotation2d()};
    wpi::array<SwerveModulePosition, 4> swerveModulePositionArray = {defaultSwervePose, defaultSwervePose, defaultSwervePose, defaultSwervePose};

    /// @TODO: May want to create swervemodule locations in a different class, call getter here and in swerve chassis

    m_poseEstimator = new SwerveDrivePoseEstimator<4>(Rotation2d(), 
                        Pose2d(),
                        swerveModulePositionArray,
                        m_kinematics,
                        {0.1, 0.1, 0.1},   // state standard deviations
                        {0.05},            // local measurement standard deviations
                        {0.1, 0.1, 0.1}); // vision measurement standard deviations
}

/// @brief update the chassis odometry based on current states of the swerve modules and the pigeon
void SwerveOdometry::UpdateOdometry() 
{
    units::degree_t yaw{PigeonFactory::GetFactory()->GetPigeon(DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT)->GetYaw()};
    Rotation2d rot2d {yaw}; 

    auto currentPose = m_poseEstimator->GetEstimatedPosition();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("Swerve Chassis"), "Odometry: Current X", currentPose.X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("Swerve Chassis"), "Odometry: Current Y", currentPose.Y().to<double>());

    m_poseEstimator->Update(rot2d, {m_frontLeft.get()->GetState(),
                                  m_frontRight.get()->GetState(), 
                                  m_backLeft.get()->GetState(),
                                  m_backRight.get()->GetState()}, 
                                    {m_frontLeft.get()->GetPosition(),
                                    m_frontRight.get()->GetPosition(), 
                                    m_backLeft.get()->GetPosition(),
                                    m_backRight.get()->GetPosition()});

    auto updatedPose = m_poseEstimator->GetEstimatedPosition();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("Swerve Chassis"), "Odometry: Updated X", updatedPose.X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("Swerve Chassis"), "Odometry: Updated Y", updatedPose.Y().to<double>());
}

Pose2d SwerveOdometry::GetPose() const
{
    return m_poseEstimator->GetEstimatedPosition();
}

/// @brief Reset the current chassis pose based on the provided pose and rotation
/// @param [in] const Pose2d&       pose        Current XY position
/// @param [in] const Rotation2d&   angle       Current rotation angle
void SwerveOdometry::ResetPose
( 
    const Pose2d&       pose,
    const Rotation2d&   angle
)
{
    m_poseEstimator->ResetPosition(pose, angle,
                                    {m_frontLeft.get()->GetPosition(),
                                    m_frontRight.get()->GetPosition(), 
                                    m_backLeft.get()->GetPosition(),
                                    m_backRight.get()->GetPosition()});
                                    
    ChassisFactory::GetChassisFactory()->GetSwerveChassis()->SetEncodersToZero();

    auto pigeon = PigeonFactory::GetFactory()->GetPigeon(DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT);

    pigeon->ReZeroPigeon(angle.Degrees().to<double>(), 0);
}

void SwerveOdometry::ResetPose
( 
    const Pose2d&       pose
)
{
    Rotation2d angle = pose.Rotation();

    ResetPose(pose, angle);
}

frc::SwerveDriveKinematics<4> SwerveOdometry::GetSwerveKinematics() const
{
    return m_kinematics;
}