
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

#include <mechanisms/MechanismFactory.h>
#include <mechanisms/MechanismTypes.h>
#include <mechanisms/StateMgr.h>
#include <mechanisms/StateMgrHelper.h>

void StateMgrHelper::RunCurrentMechanismStates() 
{
    for (auto i=MechanismTypes::MECHANISM_TYPE::UNKNOWN_MECHANISM+1; i<MechanismTypes::MECHANISM_TYPE::MAX_MECHANISM_TYPES; ++i)
    {
        auto mech = MechanismFactory::GetMechanismFactory()->GetMechanism(static_cast<MechanismTypes::MECHANISM_TYPE>(i));
        auto stateMgr = mech != nullptr ? mech->GetStateMgr() : nullptr;
        if (stateMgr != nullptr)
        {
            stateMgr->RunCurrentState();
        }
    }   
}
