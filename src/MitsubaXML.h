//
// Created by felixfifi on 19.06.20.
//

#ifndef RTX_RAYTRACER_MITSUBAXML_H
#define RTX_RAYTRACER_MITSUBAXML_H

#include "tinyxml2.h"
#include <glm/vec3.hpp>
#include <stdexcept>
#include <sstream>
#include <vector>

using namespace tinyxml2;

XMLElement *getNamedChild(XMLElement *elem, const std::string &name, const char *FILTER) {
    XMLElement *xChild = elem->FirstChildElement(FILTER);

    while (xChild) {
        const XMLAttribute *xName = xChild->FindAttribute("name");
        if (xName && xName->Value() == name) {
            break;
        }

        xChild = xChild->NextSiblingElement(FILTER);
    }
    return xChild;
}

glm::vec3 parseCommaSpaceSeparatedVec3(const std::string &text) {
    std::stringstream ss(text);

    std::vector<float> values;
    for (float f; ss >> f;) {
        values.push_back(f);
        if (ss.peek() == ',' || ss.peek() == ' ')
            ss.ignore();
    }

    if (values.size() == 1) {
        // All the same values
        return {values[0], values[0], values[0]};
    } else if (values.size() != 3) {
        throw std::runtime_error("Parsed text did not contain 3 values");
    }

    return {values[0], values[1], values[2]};
}

bool hasChildString(XMLElement *elem, const std::string &name) {
    const char *FILTER = "string";
    XMLElement *xChild = getNamedChild(elem, name, FILTER);

    return xChild != nullptr;
}

bool getChildBool(XMLElement *elem, const std::string &name) {
    const char *FILTER = "boolean";
    XMLElement *xChild = getNamedChild(elem, name, FILTER);

    if (!xChild) {
        throw std::runtime_error("Name not found: " + name);
    }

    return xChild->BoolAttribute("value");
}

float getChildFloat(XMLElement *elem, const std::string &name) {
    const char *FILTER = "float";
    XMLElement *xChild = getNamedChild(elem, name, FILTER);

    if (!xChild) {
        throw std::runtime_error("Name not found: " + name);
    }

    return xChild->FloatAttribute("value");
}

float getChildSingleSpectrum(XMLElement *elem, const std::string &name) {
    const char *FILTER = "spectrum";
    XMLElement *xChild = getNamedChild(elem, name, FILTER);

    if (!xChild) {
        throw std::runtime_error("Name not found: " + name);
    }

    return xChild->FloatAttribute("value");
}

int getChildInt(XMLElement *elem, const std::string &name) {
    const char *FILTER = "integer";
    XMLElement *xChild = getNamedChild(elem, name, FILTER);

    if (!xChild) {
        throw std::runtime_error("Name not found: " + name);
    }

    return xChild->Int64Attribute("value");
}

std::string getChildString(XMLElement *elem, const std::string &name) {
    const char *FILTER = "string";
    XMLElement *xChild = getNamedChild(elem, name, FILTER);

    if (!xChild) {
        throw std::runtime_error("Name not found: " + name);
    }

    return xChild->Attribute("value");
}


glm::vec3 getChildRGB(XMLElement *elem, const std::string &name) {
    const char *FILTER = "rgb";
    XMLElement *xChild = getNamedChild(elem, name, FILTER);

    if (!xChild) {
        throw std::runtime_error("Name not found: " + name);
    }

    return parseCommaSpaceSeparatedVec3(xChild->Attribute("value"));
}

glm::vec3 getChildPoint(XMLElement *elem, const std::string &name) {
    const char *FILTER = "point";
    XMLElement *xChild = getNamedChild(elem, name, FILTER);

    if (!xChild) {
        throw std::runtime_error("Name not found: " + name);
    }

    return {xChild->FloatAttribute("x"), xChild->FloatAttribute("y"), xChild->FloatAttribute("z")};
}


#endif //RTX_RAYTRACER_MITSUBAXML_H
