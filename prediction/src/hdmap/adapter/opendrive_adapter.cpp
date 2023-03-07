//
// Created by ai on 23-3-7.
//

#include "opendrive_adapter.h"
namespace aptiv {
namespace hdmap {
namespace adapter {

using namespace aptiv::hdmap::entity;
using namespace aptiv::hdmap::parser;

bool OpendriveAdapter::LoadData(const std::string &filename, std::shared_ptr<aptiv::hdmap::entity::Map> map) {
    tinyxml2::XMLDocument document;
    if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
        LOG(ERROR) << "fail to load file " << filename;
        return false;
    }

    //root node
    const tinyxml2::XMLElement* root_node = document.RootElement();
    std::shared_ptr<aptiv::hdmap::entity::Header> header(new aptiv::hdmap::entity::Header());
    //header
    HeaderXMlParser::Parse(*root_node,header);
    map->header = header;

    //road node
    std::vector<RoadInternal> roads;
    RoadXmlParser::Parse(*root_node,&roads);
    return true;
}
}
}
}