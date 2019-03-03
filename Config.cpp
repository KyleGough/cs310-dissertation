#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include "Config.h"
#include "CommunicationMethod.h"
using namespace std;

void Config::readConfig(vector<vector<int>> &presets, CommunicationMethod &method, float &searchR, float &commR) {

	ifstream configFile;
	string configLine;
	int lineNumber = 0;
	configFile.open("config.txt");

	cout << "Reading Config file..." << endl;

	while (!configFile.eof()) {
		lineNumber++;
		getline(configFile, configLine);

		if (configLine.size() == 0 || configLine[0] == '#') { continue; }

		vector<string> splitLine = split(configLine, ':');

		if (splitLine.size() != 2) { continue; }

		string s = splitLine[0];

		try {
			if (s == "COMM_METHOD") {
				if (splitLine[1] == "LOCAL") { method = Local; }
				if (splitLine[1] == "GLOBAL") { method = Global; }
			}
			else if (s == "P1_X") { presets[0][0] = getInt(splitLine[1]); }
			else if (s == "P1_Y") { presets[0][1] = getInt(splitLine[1]); }
			else if (s == "P1_FP") { presets[0][2] = getInt(splitLine[1]); }
			else if (s == "P1_NS") { presets[0][3] = getInt(splitLine[1]); }
			else if (s == "P1_IT") { presets[0][4] = getInt(splitLine[1]); }
			else if (s == "P2_X") { presets[1][0] = getInt(splitLine[1]); }
			else if (s == "P2_Y") { presets[1][1] = getInt(splitLine[1]); }
			else if (s == "P2_FP") { presets[1][2] = getInt(splitLine[1]); }
			else if (s == "P2_NS") { presets[1][3] = getInt(splitLine[1]); }
			else if (s == "P2_IT") { presets[1][4] = getInt(splitLine[1]); }
			else if (s == "P3_X") { presets[2][0] = getInt(splitLine[1]); }
			else if (s == "P3_Y") { presets[2][1] = getInt(splitLine[1]); }
			else if (s == "P3_FP") { presets[2][2] = getInt(splitLine[1]); }
			else if (s == "P3_NS") { presets[2][3] = getInt(splitLine[1]); }
			else if (s == "P3_IT") { presets[2][4] = getInt(splitLine[1]); }
			else if (s == "P4_X") { presets[3][0] = getInt(splitLine[1]); }
			else if (s == "P4_Y") { presets[3][1] = getInt(splitLine[1]); }
			else if (s == "P4_FP") { presets[3][2] = getInt(splitLine[1]); }
			else if (s == "P4_NS") { presets[3][3] = getInt(splitLine[1]); }
			else if (s == "P4_IT") { presets[3][4] = getInt(splitLine[1]); }
			else if (s == "P5_X") { presets[4][0] = getInt(splitLine[1]); }
			else if (s == "P5_Y") { presets[4][1] = getInt(splitLine[1]); }
			else if (s == "P5_FP") { presets[4][2] = getInt(splitLine[1]); }
			else if (s == "P5_NS") { presets[4][3] = getInt(splitLine[1]); }
			else if (s == "P5_IT") { presets[4][4] = getInt(splitLine[1]); }
			else if (s == "SEARCH_R") { searchR = getInt(splitLine[1]); }
			else if (s == "COMM_R") { commR = getInt(splitLine[1]); }
		}
		catch (const invalid_argument &e) {
			cout << "Invalid argument on Line (" << lineNumber << ")" << endl;
		}
		catch (const out_of_range &e) {
			cout << "Value out of range on Line (" << lineNumber << ")" << endl;
		}


	}

	cout << "Config file processed." << endl;
	configFile.close();
}

vector<string> Config::split(const string& s, char delimiter) {
   vector<string> tokens;
   string token;
   istringstream tokenStream(s);
   while (getline(tokenStream, token, delimiter)) {
      tokens.push_back(token);
   }
   return tokens;
}

int Config::getInt(string s) {
	return stoi(s);
}
