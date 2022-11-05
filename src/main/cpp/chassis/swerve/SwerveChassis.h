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

//C++ Includes
#include <memory>

//FRC Includes
#include <frc/geometry/Pose2d.h>

//Team302 Includes
#include <states/chassis/SwerveDriveState.h>
#include <chassis/swerve/ISwerveDriveOrientation.h>
#include <chassis/swerve/SwerveEnums.h>

class SwerveChassis
{
    public:
        SwerveChassis();

        /// @brief Runs the current SwerveDriveState
        void Drive();

        /// @brief Initializes the targetState, updates the current SwerveDriveState
        /// @param [in] SwerveDriveState    targetState - the new state that should be ran
        void Drive(SwerveDriveState* targetState);

        /// @brief Returns the current SwerveDriveState
        /// @return SwerveDriveState* - current SwerveDriveState
        SwerveDriveState* GetCurrentDriveState() const {return m_currentDriveState;};

        /// @brief Get the specified SwerveDriveState
        /// @return SwerveDriveState* - specified state
        SwerveDriveState* GetDriveState(SwerveDriveState::SwerveDriveStateType stateType);

        /// @brief Get current estimated chassis position as Pose2d
        /// @return frc::Pose2d - current chassis position
        frc::Pose2d GetPose();

        /// @brief Set the current chassis position to the target pose
        /// @param [in] frc::Pose2d pose - target pose
        void ResetPose(frc::Pose2d pose);

        /// @brief Update current estimated chassis position based on encoders and sensors
        void UpdateOdometry();

        /// @brief Get the current chassis orientation "state"
        /// @return ISwerveDriveOrientation* - current orientation
        ISwerveDriveOrientation* GetOrientation();
    
    private:
        //Dont know if this and orientation should be vectors or maps, b/c of GetDriveState: I feel like they should be maps
        //std::vector<SwerveDriveState*>          m_swerveDriveStates;
        SwerveDriveState*                                                   m_currentDriveState;

        std::map<SwerveDriveState::SwerveDriveStateType, SwerveDriveState*> m_swerveDriveStates;

        //std::vector<ISwerveDriveOrientation*>   m_swerveOrientation;
        std::map<SwerveEnums::HeadingOption, ISwerveDriveOrientation*>      m_swerveOrientation;
        ISwerveDriveOrientation*                                            m_currentOrientation;
};