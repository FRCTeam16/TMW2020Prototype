#pragma once
#include <string>
#include "Util/BSPrefs.h"

using namespace std;

class BSPrefsFilebased : public BSPrefs {
 public:
  BSPrefsFilebased();
 private:
  const string filename = "/home/lvuser/deploy/bsprefs.csv";
};
