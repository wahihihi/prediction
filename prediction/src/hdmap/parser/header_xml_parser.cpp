//
// Created by ai on 23-3-1.
//

#include "header_xml_parser.h"

namespace aptiv{
namespace hdmap{
namespace adapter{

    int HeaderXMlParser::Parse(const tinyxml2::XMLElement& node, std::shared_ptr<aptiv::hdmap::entity::Header> header) {
        auto header_node = node.FirstChildElement("header");
        std::string rev_major;

        const std::string revMajor = header_node->Attribute("revMajor");
        const std::string revMinor = header_node->Attribute("revMinor");
        const std::string name = header_node->Attribute("name");
        const std::string version = header_node->Attribute("version");
        const std::string date = header_node->Attribute("date");
        double north = 0.0;
        double south = 0.0;
        double east = 0.0;
        double west = 0.0;

        header_node->QueryDoubleAttribute("north",&north);
        header_node->QueryDoubleAttribute("south",&south);
        header_node->QueryDoubleAttribute("east",&east);
        header_node->QueryDoubleAttribute("west",&west);
        header->north = north;
        header->south = south;
        header->east  = east ;
        header->west  = west ;
        header->name  = name ;

        return  0;
    }
}
}
}