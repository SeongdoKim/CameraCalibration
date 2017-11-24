#pragma once

#include <string>

#include <boost/property_tree/ptree.hpp>

bool to_bool(std::string str);

bool isValidFile(const std::string &filename);

/**
 * Read an element from given property tree
 */
bool read(boost::property_tree::ptree& ptree, std::string option, std::string &value);
