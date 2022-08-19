#pragma once

#include <memory>
#include <string>
#include <pugixml/pugixml.hpp>
#include <frc/Encoder.h>

class ThroughBoreEncoderXmlParser
{
    public:
        ThroughBoreEncoderXmlParser() = default;

        virtual ~ThroughBoreEncoderXmlParser() = default;
        std::shared_ptr<frc::Encoder> ParseXML
        (
            std::string         networkTableName,
            pugi::xml_node throughBoreEncoderNode
        );

};