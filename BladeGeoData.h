# pragma once

#include <vector>

using namespace std;

class BladeGeoData {
public:
	vector<double> bladeRadius;
	vector<double> bladeChord;
	vector<double> bladeRelativeThickness;
	vector<double> bladeTwist;
	vector<double> bladeXt4Position;
	vector<double> bladeYt4Position;
	vector<double> bladePCBAX;
	vector<double> bladePCBAY;
	vector<double> bladeRelativeTwistAxis;
};

