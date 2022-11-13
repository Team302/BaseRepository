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

// C++ Includes
#include <algorithm>
#include <memory>

// FRC includes
#include <units/velocity.h>
#include <units/angular_velocity.h>

// Team 302 Includes
#include <chassis/swerve/SwerveDrive.h>
#include <hw/DragonPigeon.h>
#include <gamepad/IDragonGamePad.h>
#include <TeleopControl.h>
#include <mechanisms/base/IState.h>
#include <chassis/ChassisFactory.h>
#include <hw/factories/PigeonFactory.h>
#include <utils/Logger.h>
#include <chassis/swerve/SwerveEnums.h>

using namespace std;
using namespace frc;

/// @brief initialize the object and validate the necessary items are not nullptrs
SwerveDrive::SwerveDrive() : IState(),
                             m_chassis(ChassisFactory::GetChassisFactory()->GetSwerveChassis()),
                             m_controller(TeleopControl::GetInstance()),
                             m_usePWLinearProfile(false),
                             m_chassisMovement(ChassisMovement{})
{
    if (m_controller == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("SwerveDrive"), string("Constructor"), string("TeleopControl is nullptr"));
    }

    if (m_chassis.get() == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("SwerveDrive"), string("Constructor"), string("Chassis is nullptr"));
    }
}

/// @brief initialize the profiles for the various gamepad inputs
/// @return void
void SwerveDrive::Init()
{
    auto controller = GetController();
    if (controller != nullptr)
    {
        auto profile = (m_usePWLinearProfile) ? IDragonGamePad::AXIS_PROFILE::PIECEWISE_LINEAR : IDragonGamePad::AXIS_PROFILE::CUBED;
        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE, profile);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE, -0.6);

        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER, profile);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER, -0.6);

        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE, profile);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE, 0.5);
    }
}

/// @brief calculate the output for the wheels on the chassis from the throttle and steer components
/// @return void
void SwerveDrive::Run()
{
    auto controller = GetController();
    if (controller != nullptr)
    {
        SwerveEnums::SwerveDriveStateType mode = SwerveEnums::SwerveDriveStateType::FIELD_DRIVE;
        SwerveEnums::HeadingOption headingOpt = SwerveEnums::HeadingOption::MAINTAIN;
        SwerveEnums::NoMovementOption stopOption = SwerveEnums::NoMovementOption::STOP;
        if (controller->IsButtonPressed(TeleopControl::FINDTARGET))
        {
            headingOpt = SwerveEnums::HeadingOption::TOWARD_GOAL;
        }                                       
        
        if (controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::REZERO_PIGEON))
        {
            auto factory = PigeonFactory::GetFactory();
            auto m_pigeon = factory->GetPigeon(DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT);
            m_pigeon->ReZeroPigeon(0, 0);
            m_chassis.get()->ZeroAlignSwerveModules();

            //Set stored yaw for orientation options to 0
            m_chassis.get()->GetOrientation(headingOpt)->SetStoredHeading(units::angle::degree_t(0.0));
        }

        if (controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::HOLD_POSITION))
        {
            stopOption = SwerveEnums::NoMovementOption::HOLD_POSITION;
            mode = SwerveEnums::SwerveDriveStateType::STOP_DRIVE;
        }
        else
        {
            stopOption = SwerveEnums::NoMovementOption::STOP;
        }

        auto drive = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE);
        auto steer = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER);
        auto rotate = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE);

        //Get selected drive mode state
        SwerveDriveState* targetState = m_chassis.get()->GetDriveState(mode);
        ISwerveDriveOrientation* targetOrientation = m_chassis.get()->GetOrientation(headingOpt);

        frc::ChassisSpeeds chassisSpeeds = {m_chassis.get()->GetMaxSpeed()  *drive,
                                            m_chassis.get()->GetMaxSpeed() *steer,
                                            m_chassis.get()->GetMaxAngularSpeed()  *rotate};
        
        //Convert all values into a ChassisMovement struct
        ChassisMovement chassisMovement = {chassisSpeeds, 
                                            *new frc::Trajectory(), 
                                            *new Point2d(),
                                            stopOption,
                                            SwerveEnums::AutonControllerType::HOLONOMIC};

        targetState->UpdateChassisMovement(chassisMovement);
        targetState->UpdateOrientationOption(*targetOrientation);

        m_chassis->Drive(targetState);
    }
}

void SwerveDrive::Exit()
{
}

/// @brief indicates that we are not at our target
/// @return bool
bool SwerveDrive::AtTarget() const
{
    return false;
}