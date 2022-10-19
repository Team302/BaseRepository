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
#include <frc/geometry/Rotation2d.h>

//Team302 Includes
#include <states/chassis/HoldDrive.h>

std::array<frc::SwerveModuleState*, 4> HoldDrive::CalcSwerveModuleStates()
{
    frc::SwerveModuleState* flState = new frc::SwerveModuleState();
    frc::SwerveModuleState* frState = new frc::SwerveModuleState();
    frc::SwerveModuleState* blState = new frc::SwerveModuleState();
    frc::SwerveModuleState* brState = new frc::SwerveModuleState();

    flState->angle = {units::angle::degree_t(45)};
    frState->angle = {units::angle::degree_t(-45)};
    blState->angle = {units::angle::degree_t(135)};
    brState->angle = {units::angle::degree_t(-135)};

    return {flState, frState, blState, brState};
}

void HoldDrive::Init()
{
    // Do we need to do anything here?
}