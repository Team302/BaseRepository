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
#include <memory>
#include <iostream>
#include <fstream>

//FRC Includes
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/TableEntryListener.h>
#include <frc/Filesystem.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>

//Team302 Includes
#include <utils/Logger.h>
#include <utils/LiveXMLTuner.h>

using namespace pugi;

LiveXMLTuner::LiveXMLTuner()
{
    frc::Shuffleboard::GetTab("LiveXML");

    frc::Shuffleboard::GetTab("LiveXML").Add("Enable LIVE XML Editing?", false).WithWidget(frc::BuiltInWidgets::kToggleButton).GetEntry();

    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

    std::shared_ptr<nt::NetworkTable> shuffleboardTable = inst.GetTable("Shuffleboard");
    m_liveXmlTable = shuffleboardTable.get()->GetSubTable("LiveXML");

    m_enableButton = m_liveXmlTable.get()->GetEntry("Enable LIVE XML Editing?");
}

void LiveXMLTuner::ListenForUpdates()
{
    if(m_enableButton.GetValue().get()->GetBoolean() == true)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("LiveXMLTuner"), std::string("Listen"), "Button pressed");
        PopulateNetworkTable();
        m_enableButton.SetBoolean(false);
    }

    //Explanation of below code
    //nt::EntryListener is an std::function object
    //This is used to create lambda functions, or code that can be assigned to variables
    //"this" is enclosed in square brackets to allow LiveXMLTuner methods to be called inside this function
    //the parantheses that follow are just the signature of the nt::EntryListener function
    //code inside the curly brackets is ran when the value changes
    //NT_NOTIFY_NEW AND NT_NOTIFY_UPDATE are just flags for when this listener executes

    m_liveXmlTable.get()->AddEntryListener([this] (nt::NetworkTable* table, 
                                                std::string_view name,
                                                nt::NetworkTableEntry entry,
                                                std::shared_ptr<nt::Value> value, 
                                                int flags) {

    
    }, NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);
}

void LiveXMLTuner::PopulateNetworkTable()
{
    auto deployDir = frc::filesystem::GetDeployDirectory();
    std::string filename = deployDir + std::string("/robot.xml");

    try
    {
        xml_document doc;
        xml_parse_result result = doc.load_file(filename.c_str());

        //If parse is good
        if (result)
        {
            if (!CreateCopyOfXML())
            {
                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, std::string("LiveXMLTuner"), std::string("PopulateNetworkTable"), std::string("Could not create copy of robot.xml, DO NOT EDIT"));
            }
            else
            {
                /// Currently only doing mechanisms and chassis
                // get the root node <robot>
                xml_node parent = doc.root();
                for (xml_node node = parent.first_child(); node; node = node.next_sibling())
                {
                    // loop through the direct children of <robot> and add to appropriate network table
                    for (xml_node child = node.first_child(); child; child = child.next_sibling())
                    {
                        if (std::strcmp(child.name(), "chassis") == 0)
                        {
                            ChassisPopulate(child);  
                        }
                        else if (std::strcmp(child.name(), "mechanism") == 0)
                        {
                            MechanismPopulate(child);
                        }
                    }
                }
            }
        }
    }    
    catch(const std::exception& e)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, std::string("LiveXMLTuner"), std::string("PopulateNetworkTable"), std::string("Error thrown while parsing robot.xml"));
    }
}

void LiveXMLTuner::ChassisPopulate(xml_node chassisNode)
{
    //create chassis table
    std::shared_ptr<nt::NetworkTable> chassisTable = m_liveXmlTable.get()->GetSubTable(std::string(chassisNode.first_attribute().value()) + " chassis");
    
    //add attributes to table
    for (xml_attribute attr = chassisNode.first_attribute(); attr; attr = attr.next_attribute())
    {
        std::string attrName (attr.name());

        if (  attrName.compare("wheelBase") == 0 )
        {
            chassisTable.get()->PutNumber("wheelBase (inches)", attr.as_double());
        }
        else if (  attrName.compare("track") == 0 )
        {
            chassisTable.get()->PutNumber("track (inches)", attr.as_double());
        }
        else if (  attrName.compare("maxVelocity") == 0 )
        {
            units::velocity::feet_per_second_t fps(attr.as_double()/12.0);
            auto maxVelocity = units::velocity::meters_per_second_t(fps);
            chassisTable.get()->PutNumber("maxVelocity (meters per second)", maxVelocity.to<double>());
        }
        else if (  attrName.compare("maxAngularVelocity") == 0 )
        {
            units::degrees_per_second_t degreesPerSec(attr.as_double());
            auto maxAngularSpeed = units::radians_per_second_t(degreesPerSec);
            chassisTable.get()->PutNumber("maxAngularSpeed (radians per second)", maxAngularSpeed.to<double>());
        }
        else if (  attrName.compare("maxAcceleration") == 0 )
        {
            auto maxAcceleration = units::feet_per_second_t(attr.as_double()/12.0) / 1_s;
            units::acceleration::meters_per_second_squared_t maxAccelMPS = maxAcceleration;
            chassisTable.get()->PutNumber("maxAcceleration (meters per second squared)", maxAccelMPS.to<double>());
        }
        else if (  attrName.compare("maxAngularAcceleration") == 0 )
        {
            auto maxAngularAcceleration = units::degrees_per_second_t(attr.as_double()) / 1_s;
            units::angular_acceleration::radians_per_second_squared_t maxAnglAccelRad = maxAngularAcceleration;
            chassisTable.get()->PutNumber("maxAngularAcceleration (radians per second squared)", maxAnglAccelRad.to<double>());
        }
        else if (  attrName.compare("wheelDiameter") == 0 )
        {
            chassisTable.get()->PutNumber("wheelDiameter (inches)", attr.as_double());
        }
        else if ( attrName.compare("odometryComplianceCoefficient") == 0 )
        {
            chassisTable.get()->PutNumber("odometryComplianceCoefficient", attr.as_double());
        }
        else if (attrName.compare("networkTable") == 0)
        {
            chassisTable.get()->PutString("networktable", attr.as_string());
        }
        else if (attrName.compare("controlFile") == 0)
        {
            chassisTable.get()->PutString("controlFile", attr.as_string());
        }
    }

    //populate swerve modules and any excess motors
    for (xml_node child = chassisNode.first_child(); child; child = child.next_sibling())
    {
        std::string childName (child.name());

        if (childName.compare("motor") == 0)
        {
            MotorPopulate(child, chassisTable);
        }
        else if (childName.compare("swervemodule") == 0)
        {
            SwerveModulePopulate(child, chassisTable);
        }
    }
}

void LiveXMLTuner::MechanismPopulate(xml_node mechNode)
{
    //create mechanism table
    std::shared_ptr<nt::NetworkTable> mechTable = m_liveXmlTable.get()->GetSubTable(std::string(mechNode.first_attribute().value()));
    
    //add attributes to table
    for (xml_attribute attr = mechNode.first_attribute(); attr; attr = attr.next_attribute())
    {
        std::string attrName (attr.name());
        if (attrName.compare("type") == 0)
        {
            std::string typeStr = attr.as_string();
        }

        for (xml_node child = mechNode.first_child(); child; child = child.next_sibling())
        {
            if ( strcmp( child.name(), "motor") == 0 )
            {
                MotorPopulate(child, mechTable);
            }
            else if ( strcmp( child.name(), "canCoder" ) == 0)
            {
            CancoderPopulate(child, mechTable);
            }
        }
    }
}

void LiveXMLTuner::MotorPopulate(xml_node motorNode, std::shared_ptr<nt::NetworkTable> nt)
{

}

void LiveXMLTuner::CancoderPopulate(xml_node cancoderNode, std::shared_ptr<nt::NetworkTable> nt)
{

}

void LiveXMLTuner::SwerveModulePopulate(xml_node moduleNode, std::shared_ptr<nt::NetworkTable> nt)
{

}

bool LiveXMLTuner::CreateCopyOfXML()
{
    bool result = false;
    
    //Create a copy of robot.xml in deploy directy, timestamped and in backups folder
    auto deployDir = frc::filesystem::GetDeployDirectory();
    std::string filename = deployDir + std::string("/robot.xml");
    /*
    //Creates directory deployDir/backups and -p just suppresses errors
    //This directory will only be created once, this command is here in case the deploy directory
    //does not have a backups folder
    std::string commandStr = "mkdir " + deployDir + "/backups -p";
    const char *command = commandStr.c_str();
    if (std::system(command) == 0)
    {
        //copy robot.xml to backups and timestamp it
        //timestamp will be easier in c++ 20, otherwise in the beta
        //for now, only keep latest backup
        commandStr = "cp " + filename + " " + deployDir + "/backups/robot.xml";
        command = commandStr.c_str();

        if (std::system(command) == 0)
        {
            result = true;
        }
    }*/

    std::ifstream in (filename.c_str());

    std::string outputFile = deployDir + "/backups/robot.xml";

    std::ofstream out (outputFile.c_str());

    out << in.rdbuf();

    out.close();
    in.close();

    result = true;

    return result;
}