# pragma once

#include <vector>

#include "AirfoilGeoData.h"

using namespace std;

class AirfoilGeoSummaryData {
public:
	vector<string> AirfoilGeoDataFileNameList;
	vector<AirfoilGeoData*> AirfoilGeoDataList;
	vector<AirfoilGeoData*> AirfoilInterpolatedGeoDataList;
};
