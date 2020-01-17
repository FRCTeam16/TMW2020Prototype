#include "Util/BSPrefsFilebased.h"
#include "Util/StringUtil.h"
#include <iostream>
#include <fstream>
#include <sstream>


// Helper function for converting a string to a double
template <class T>
bool from_string(T& t, 
                 const std::string& s, 
                 std::ios_base& (*f)(std::ios_base&))
{
  std::istringstream iss(s);
  return !(iss >> f >> t).fail();
}

BSPrefsFilebased::BSPrefsFilebased() {

    ifstream prefFile(filename);
    if (!prefFile) {
        cerr << "*************************************************\n"
             << " BSPF: Unable to read preferences file: " << filename << "\n"
             << "*************************************************\n";
             return;
    }

    if (prefFile.is_open()) {
        string key;
        string valueString;
        double value = 0.0;

        int propertiesRead = 0;
        while (!prefFile.eof()) {
            getline(prefFile, key, ',');
            getline(prefFile, valueString);

            // From RAWC: Handle weird newline situatoins
            if (prefFile.eof()) {
                break;
            }

            StringUtil::trim(key);
            StringUtil::trim(valueString);

            if (from_string<double>(value, valueString, std::dec)) {
                lookupDouble[key] = value;
                propertiesRead++;
            } else {
                cerr << "*************************************************\n"
                     << " BSPF: Unable to convert: " << key << " | " << valueString << "\n"
                     << "*************************************************\n";
            }
        }
        prefFile.close();
        cout << "BSPrefsFileBased: # of properties read: " << propertiesRead << "\n";
    } else {
        cerr << "*************************************************\n"
             << " BSPF: Unable to open preferences file: " << filename << "\n"
             << "*************************************************\n";
             return;
    }
}
