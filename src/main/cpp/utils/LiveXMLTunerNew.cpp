//C++ Includes
#include <memory>
#include <iostream>
#include <fstream>

//FRC Includes
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
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

    m_enableButton = frc::Shuffleboard::GetTab("LiveXML").Add("Enable LIVE XML Editing?", false).WithWidget(frc::BuiltInWidgets::kToggleButton).GetEntry();
    m_submitButton = frc::Shuffleboard::GetTab("LiveXML").Add("Submit Changes?", false).WithWidget(frc::BuiltInWidgets::kToggleButton).GetEntry();

    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

    std::shared_ptr<nt::NetworkTable> shuffleboardTable = inst.GetTable("Shuffleboard");
    m_liveXmlTable = shuffleboardTable.get()->GetSubTable("LiveXML");
}

void LiveXMLTuner::ListenForUpdates()
{
    if(m_enableButton.GetValue().get()->GetBoolean())
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("LiveXMLTuner"), std::string("Listen"), "Button pressed");
        PopulateNetworkTable();
        m_enableButton.SetBoolean(false);
    }

    if(m_submitButton.GetValue().get()->GetBoolean())
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("LiveXMLTuner"), std::string("Submit"), "Button pressed");
        ModifyElements(m_liveXmlTable);
        m_submitButton.SetBoolean(false);
    }
    

    //Explanation of below code
    //nt::EntryListener is an std::function object
    //This is used to create lambda functions, or code that can be assigned to variables
    //"this" is enclosed in square brackets to allow LiveXMLTuner methods to be called inside this function
    //the parantheses that follow are just the signature of the nt::EntryListener function
    //code inside the curly brackets is ran when the value changes
    //NT_NOTIFY_NEW AND NT_NOTIFY_UPDATE are just flags for when this listener executes

    ///
    /// May not use entry listeners, may instead just push all changes when the "Submit changes" button is pressed.
    ///

    // m_liveXmlTable.get()->AddEntryListener([this] (nt::NetworkTable* table, 
    //                                             std::string_view name,
    //                                             nt::NetworkTableEntry entry,
    //                                             std::shared_ptr<nt::Value> value, 
    //                                             int flags) {
    //     //std::shared_ptr<nt::NetworkTable> tablePtr(table);
    //     //ModifyElements(m_liveXmlTable);
    //     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, std::string("LiveXMLTuner"), std::string("EntryListener"), std::string("Edited " + entry.GetName()));
    
    // }, NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);
}

void LiveXMLTuner::PopulateNetworkTable()
{
    //auto deployDir = frc::filesystem::GetDeployDirectory();
    //std::string filename = deployDir + std::string("/robot.xml");

    /// DEBUG
    std::string deployDir = "D:\\Github\\Testing";
    std::string filename = deployDir + std::string("\\robot.xml");
    /// DEBUG

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
        //currently we just put up the attribute value as a string
        //may create a switch statement to put up as double, string, and bool to plot or change
        std::string value = attr.as_string();

        //sends attribute value to network table
        chassisTable.get()->PutString(attr.name(), value);

        //updates entry map to have attribute and entry correlate
        m_entryAttributeMap[chassisTable.get()->GetHierarchy(attr.name()).back()] = attr;
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
    //auto deployDir = frc::filesystem::GetDeployDirectory();
    //std::string filename = deployDir + std::string("/robot.xml");
    
    /// DEBUG
    /// This is here so I can test at home
    std::string deployDir = "D:\\Github\\Testing";
    std::string filename = deployDir + std::string("\\robot.xml");
    /// DEBUG


    std::ifstream in (filename.c_str());

    if(in.fail() || !in.is_open())
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, std::string("LiveXMLTuner"), std::string("CreateCopy"), std::string("Could not open file stream to xml"));
    }

    std::string outputFile = deployDir + "\\robotsecond.xml";

    std::ofstream out (outputFile.c_str());

    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, std::string("LiveXMLTuner"), std::string("OutputFile"), outputFile);

    out << in.rdbuf();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, std::string("LiveXMLTuner"), std::string("CreateCopy"), in.rdbuf());

    out.close();
    in.close();

    result = true;

    return result;
}

void LiveXMLTuner::ModifyElements(std::shared_ptr<nt::NetworkTable> nt)
{
    //get the next level of tables, this will be things like chassis and indepedent mechanisms
    std::vector<std::string> subTables = nt.get()->GetSubTables();

    //string to make sure we aren't modifying multiple times
    std::string lastTable = "";
    
    for(int i = 0; i < subTables.size(); i++)
    {
        if(!(subTables[i] == lastTable))
        {
            //check if there are lower tables in hierarchy
            if(!nt.get()->GetSubTable(subTables[i]).get()->GetSubTables().empty())
            {
                ModifyElements(nt);
                std::cout << subTables[i] << "traversing down" << std::endl;
            }
            else
            {
                ModifyXml(nt, nt.get()->GetKeys());
                std::cout << subTables[i] << " modifying xml" << std::endl;
            }
            lastTable = subTables[i];
        }
    }
}

bool LiveXMLTuner::ModifyXml(std::shared_ptr<nt::NetworkTable> nt, std::vector<std::string> keys)
{
    std::vector<std::string> hiearchy = nt.get()->GetHierarchy(keys.front());

    //Gets the attribute path like /Shuffleboard/LiveXML/SWERVE chassis/track (inches) 
    std::string attributeStr = hiearchy.back();
    std::cout << attributeStr << std::endl;
    
    //If the network table entry value and the stored xml value don't match, update xml value
    
    switch(nt.get()->GetValue(keys.front()).get()->type())
    {
        case NT_Type::NT_STRING:
            if(nt.get()->GetValue(keys.front())->GetString() != m_entryAttributeMap[attributeStr].as_string());
            {
                m_entryAttributeMap[attributeStr].set_value(std::string{nt.get()->GetValue(keys.front())->GetString()}.c_str());
            }  
            break;
        
        case NT_Type::NT_DOUBLE:
            if(nt.get()->GetValue(keys.front())->GetDouble() != m_entryAttributeMap[attributeStr].as_double());
            {
                m_entryAttributeMap[attributeStr].set_value(nt.get()->GetValue(keys.front())->GetDouble());
            }
            break;
        
        case NT_Type::NT_BOOLEAN:
            if(nt.get()->GetValue(keys.front())->GetBoolean() != m_entryAttributeMap[attributeStr].as_bool());
            {
                m_entryAttributeMap[attributeStr].set_value(nt.get()->GetValue(keys.front())->GetBoolean());
            }
            break;

        default:
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, std::string("LiveXMLTuner"), std::string("ModifyXml"), std::string("Network Table entry: " + nt.get()->GetEntry(keys.front()).GetName() + " does not have support type."));
            break;
    }
    
      
    /// DEBUG
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, std::string("LiveXMLTuner"), std::string("ModifyXml"), std::string("Edited XML"));  
    
    return true;
}

nt::NetworkTableEntry LiveXMLTuner::FindAttribute(std::string path)
{
    std::string attributeStr = "";

    //Later I'll have to get this by parsing through path, rn it only works for chassis
    std::string chassisString = "chassis/";

    std::string testString = "";

    for(int i = 0; i<path.length(); i++)
    {
        if(i > path.find(chassisString) + chassisString.length() - 1)
        {
            attributeStr+= path.at(std::size_t(i));
        }
    }

    return m_chassisTable.get()->GetEntry(attributeStr);

    /// @TODO
    //Pair by path and value
}