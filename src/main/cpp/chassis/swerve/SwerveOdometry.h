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

#pragma once

//FRC Includes
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

//Team302 Includes
#include <chassis/ChassisFactory.h>

class SwerveOdometry
{
    public:
        /// @brief update the chassis odometry based on current states of the swerve modules and the pigeon
        void UpdateOdometry();

        /// @brief Reset the current chassis pose based on the provided pose and rotation
        /// @param [in] const Pose2d&       pose        Current XY position
        /// @param [in] const Rotation2d&   angle       Current rotation angle
        void ResetPose
        ( 
            const frc::Pose2d&       pose,
            const frc::Rotation2d&   angle
        );

        /// @brief Reset the current chassis pose based on the provided pose (the rotation comes from the Pigeon)
        /// @param [in] const Pose2d&       pose        Current XY position
        void ResetPose
        ( 
            const frc::Pose2d&       pose
        );

        /// @brief Get the current chassis pose
        frc::Pose2d GetPose() const;

    private:
        SwerveChassis* m_chassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();

        frc::Translation2d m_frontLeftLocation;
        frc::Translation2d m_frontRightLocation;
        frc::Translation2d m_backLeftLocation;
        frc::Translation2d m_backRightLocation;

        std::shared_ptr<SwerveModule>                               m_frontLeft;
        std::shared_ptr<SwerveModule>                               m_frontRight;
        std::shared_ptr<SwerveModule>                               m_backLeft;
        std::shared_ptr<SwerveModule>                               m_backRight;

        frc::SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, 
                                                   m_frontRightLocation, 
                                                   m_backLeftLocation, 
                                                   m_backRightLocation};

        // Gains are for example purposes only - must be determined for your own robot!
        //Clean up to get clearer information
        frc::SwerveDrivePoseEstimator<4> m_poseEstimator{  frc::Rotation2d(), 
                                                           frc::Pose2d(), 
                                                           m_kinematics,
                                                           {0.1, 0.1, 0.1},   // state standard deviations
                                                           {0.05},            // local measurement standard deviations
                                                           {0.1, 0.1, 0.1} }; // vision measurement standard deviations
};