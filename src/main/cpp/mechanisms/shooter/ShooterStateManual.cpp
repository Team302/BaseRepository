//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302 
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================


// C++ Includes

// FRC includes

// Team 302 includes
#include <TeleopControl.h>
#include <basemechanisms/interfaces/IState.h>
#include <mechanisms/shooter/ShooterStateManual.h>
#include <mechanisms/MechanismFactory.h>
#include <mechanisms/shooter/Shooter.h>

// Third Party Includes


ShooterStateManual::ShooterStateManual() : IState(), 
                                           m_shooter(MechanismFactory::GetMechanismFactory()->GetShooter()),
                                           m_controller(TeleopControl::GetInstance())
{
}

void ShooterStateManual::Init() 
{
}
void ShooterStateManual::Run() 
{
    //if (m_controller != nullptr && m_shooter != nullptr)
    //{
    //    auto shooterHoodAdjust = m_controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_HOOD_MAN);
    //}
}
void ShooterStateManual::Exit()
{
}

bool ShooterStateManual::AtTarget() const 
{
    return true;
}
