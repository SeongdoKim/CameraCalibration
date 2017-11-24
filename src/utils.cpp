#include <iostream>
#include <sstream>
#include <boost/filesystem.hpp>

#include "utils.h"

using namespace std;

bool to_bool(string str) {
	transform(str.begin(), str.end(), str.begin(), ::tolower);
	istringstream is(str);
	bool b;
	is >> std::boolalpha >> b;
	return b;
}

bool isValidFile(const string &filename) {
	boost::filesystem::path p(filename);
	if (!boost::filesystem::exists(p)) {
		cout << "Custom setting file \'" << filename << "\' does not exists" << endl;
		return false;
	}
	if (!boost::filesystem::is_regular_file(p)) {
		cout << "Invalid setting file type \'" << filename << "\'" << endl;
		return false;
	}
	return true;
}

bool read(boost::property_tree::ptree& ptree, std::string option, std::string &value) {
	value = ptree.get<std::string>(option, "");
	if (value.empty()) {
		cout << "Failed to read necessary option \'" << option << "\'" << endl;
		return false;
	}
	return true;
}
