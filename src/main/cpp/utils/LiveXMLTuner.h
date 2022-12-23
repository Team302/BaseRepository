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
#include <string>
#include <memory>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <map>

//Third Party Includes
#include <pugixml/pugixml.hpp>

class LiveXMLTuner
{
    public:
        LiveXMLTuner();
        ~LiveXMLTuner() = default;

        /// @brief Listen for updates to network table
        void ListenForUpdates();
    
    private:
        /// @brief Populate network table with all the xml elements like chassis and mechanism, and their respective attributes
        void PopulateNetworkTable();

        /// @brief Populates the chassis group with chassis values
        void ChassisPopulate(pugi::xml_node chassisNode);

        /// @brief Populates the mechanism group with mechanism values
        void MechanismPopulate(pugi::xml_node mechNode);

        /// @brief Populates the motor group with motor values
        void MotorPopulate(pugi::xml_node motorNode, std::shared_ptr<nt::NetworkTable> nt);

        /// @brief Populates the swerve module group with swerve module values
        void SwerveModulePopulate(pugi::xml_node moduleNode, std::shared_ptr<nt::NetworkTable> nt);

        /// @brief Populates the can coder group with can coder values
        void CancoderPopulate(pugi::xml_node cancoderNode, std::shared_ptr<nt::NetworkTable> nt);

        /// @brief Backs up the current robot.xml in case of breaking changes or want to revert changes
        bool CreateCopyOfXML();

        /// @brief Finds the keys of an element in the network table
        void ModifyElements(std::shared_ptr<nt::NetworkTable> nt);

        /// @brief Find xml_attribute given a network table path
        /// @return pugI::xml_attribute - The specified attribute
        //std::pair<nt::NetworkTableEntry, pugi::xml_attribute> FindAttribute(std::string path);
        nt::NetworkTableEntry FindAttribute(std::string path);

        /// @brief Modifies the xml file with elements found from FindElements()
        /// @return bool - Success
        bool ModifyXml(std::shared_ptr<nt::NetworkTable> nt, std::vector<std::string> keys);

        std::shared_ptr<nt::NetworkTable>   m_liveXmlTable;
        nt::NetworkTableEntry               m_enableButton;

        std::map<std::string, pugi::xml_attribute> m_entryAttributeMap;

        std::shared_ptr<nt::NetworkTable> m_chassisTable;
};