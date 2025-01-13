# pragma once

#include <vector>
#include <string>

using namespace std;

class ProjectData {
public:
	bool takeAnalyseSectionsInput;
	double hubRadius;
	double bladeLength;
	string airfoilGeoSummaryFileName;
	string airfoilPerfoSummaryFileName;
	string bladeGeoFileName;
	vector<double> bladeAnalyseSections;
	vector<double> bladeSegmentWidth;;
	vector<double> relThicknessAtAnalyseSections;
};
