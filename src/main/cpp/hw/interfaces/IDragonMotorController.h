
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

// C++ Includes
#include <map>
#include <memory>
#include <string>

// FRC includes
#include <frc/motorcontrol/MotorController.h>

// Team 302 includes
#include <hw/usages/MotorControllerUsage.h>
#include <mechanisms/controllers/ControlModes.h>
#include <mechanisms/controllers/ControlData.h>

// Third Party Includes
#include <ctre/phoenix/motorcontrol/RemoteSensorSource.h>
#include <ctre/phoenix/motorcontrol/StatusFrame.h>

/// @interface IDragonMotorController
/// @brief The general interface to motor mechanisms/controllers so that the specific mechanisms that use motors,
///        don't need to special case what motor controller is being used.
class IDragonMotorController
{
    public:

        enum MOTOR_PRIORITY
        {
            HIGH,
            MEDIUM,
            LOW
        };

        enum MOTOR_TYPE
        {
            UNKNOWN_MOTOR=-1,
            FALCON500,
            NEOMOTOR,
            NEO500MOTOR,  
            CIMMOTOR,
            MINICIMMOTOR,
            BAGMOTOR, 
            PRO775,
            ANDYMARK9015,
            ANDYMARKNEVEREST,
            ANDYMARKRS775125,
            ANDYMARKREDLINEA, 
            REVROBOTICSHDHEXMOTOR,
            BANEBOTSRS77518V,
            BANEBOTSRS550,
            MODERNROBOTICS12VDCMOTOR,
            JOHNSONELECTRICALGEARMOTOR,
            TETRIXMAXTORQUENADOMOTOR,
            NONE,
            MAX_MOTOR_TYPES
        };
        // Getters
        /// @brief  Return the number of revolutions the output shaft has spun
        /// @return double number of revolutions
        virtual double GetRotations() const = 0;

        /// @brief  Return the angular velocity in revolutions per second
        /// @return double angular velocity in revolutions per second
        virtual double GetRPS() const = 0;

        /// @brief  Return the usage of the motor
        /// @return MotorControllerUsage::MOTOR_CONTROLLER_USAGE - what the motor is used for
        virtual MotorControllerUsage::MOTOR_CONTROLLER_USAGE GetType() const = 0;
        
        /// @brief  Return the type of  motor
        /// @return MotorControllerUsage::MOTOR_CONTROLLER_USAGE - what the motor type is
        virtual IDragonMotorController::MOTOR_TYPE GetMotorType() const = 0;

        /// @brief  Return the current usage
        /// @return double - amperage usage for the controller
        virtual double GetCurrent() const = 0;

        /// @brief  Return the CAN ID
        /// @return int - CAN ID
        virtual int GetID() const = 0;

        /// @brief  Return the speedcontroller 
        /// @return std::shared_ptr<frc::SpeedControll> - pointer to the speed controller object
        virtual std::shared_ptr<frc::MotorController> GetSpeedController() const = 0;

        // Setters
        virtual void Set(double value) = 0;
        virtual void SetRotationOffset(double rotations) = 0;
        virtual void SetVoltageRamping(double ramping, double closedLoopRamping = -1) = 0;
        virtual void EnableCurrentLimiting(bool enabled) = 0;
        virtual void EnableBrakeMode(bool enabled) = 0;
        virtual void Invert(bool inverted) = 0;
        virtual void SetSensorInverted(bool inverted) = 0;
		virtual void SetDiameter( double diameter ) = 0;
        virtual void SetVoltage(  units::volt_t output ) = 0;
        //virtual ControlModes::CONTROL_TYPE GetControlMode() const = 0;
        virtual double GetCounts() const = 0;


        /// @brief  Set the control constants (e.g. PIDF values).
        /// @param [in] int             slot - hardware slot to use
        /// @param [in] ControlData*    pid - the control constants
        /// @return void
        virtual void SetControlConstants(int slot, ControlData* controlInfo) = 0;

        virtual void SetRemoteSensor
        (
            int                                             canID,
            ctre::phoenix::motorcontrol::RemoteSensorSource deviceType
        ) = 0;
        virtual void UpdateFramePeriods
        (
	        ctre::phoenix::motorcontrol::StatusFrameEnhanced	frame,
            uint8_t			                                    milliseconds
        ) = 0;

        
        virtual void SetFramePeriodPriority
        (
            MOTOR_PRIORITY              priority
        ) = 0;
            
        virtual double GetCountsPerRev() const = 0;
        
        IDragonMotorController() = default;
        virtual ~IDragonMotorController() = default;
        virtual double GetGearRatio() const = 0;

        virtual bool IsForwardLimitSwitchClosed() const = 0;
        virtual bool IsReverseLimitSwitchClosed() const = 0;
        virtual void EnableVoltageCompensation( double fullvoltage) = 0;

        virtual void SetSelectedSensorPosition
        (
            double  initialPosition
        ) = 0;
        virtual double GetCountsPerInch() const = 0;
        virtual double GetCountsPerDegree() const = 0;
        virtual void EnableDisableLimitSwitches
        (
            bool enable
        ) = 0;

    protected:

};

