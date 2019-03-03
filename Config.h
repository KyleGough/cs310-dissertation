#pragma once
#include <vector>
#include <string>
#include "CommunicationMethod.h"
using namespace std;

class Config {
public:
  static void readConfig(vector<vector<int>> &presets, CommunicationMethod &method, float &searchR, float &commR);
private:
  static vector<string> split(const string& s, char delimiter);
  static int getInt(string s);
};
