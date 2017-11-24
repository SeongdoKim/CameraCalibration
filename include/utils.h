#pragma once

#include <string>

#include <boost/property_tree/ptree.hpp>

bool to_bool(std::string str);

bool isValidFile(const std::string &filename);

/**
 * Check if given folder is exists.
 */
bool checkFolder(const std::string folderpath);

/**
 * Read an element from given property tree
 */
bool read(boost::property_tree::ptree& ptree, std::string option, std::string &value);
