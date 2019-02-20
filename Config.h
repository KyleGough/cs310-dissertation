#pragma once
#include <vector>
#include <string>
using namespace std;

class Config {
public:
  static void readConfig(vector<vector<int>> &presets);
private:
  static vector<string> split(const string& s, char delimiter);
  static int getInt(string s);
};
