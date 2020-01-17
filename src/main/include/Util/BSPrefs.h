#pragma once

#include <string>
#include <map>
#include "Util/RAWCConstants.h"

class BSPrefs {
public:
    static BSPrefs* GetInstance();

    double GetDouble(std::string key, double value) const;
    int GetInt(std::string key, int value) const;
    bool GetBool(std::string key, bool value) const;

    void StoreDouble(std::string key, double value);
    void SaveConstants();


private:
    static BSPrefs* instance;

protected:
    BSPrefs() = default;
    void LoadConstants();
    std::map<std::string, double> lookupDouble;
    std::map<std::string, bool> lookupBool;
    RAWCConstants *constants;
};

class BSPrefsHardcoded : public BSPrefs {
public:
    BSPrefsHardcoded();
};