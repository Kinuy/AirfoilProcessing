# pragma once

#include <vector>
#include <string>
#include <cmath>

#include "Node.h"
#include "ProjectData.h"

using namespace std;

class AirfoilGeoData {
public:
	int markerTEE;
	int markerTESSMAX;
	int markerTESS;
	int markerLESSMAX;
	int markerLE;
	int markerLEPSMAX;
	size_t markerTEPS;
	size_t markerTEPSMAX;
	size_t markerTE;
	int markerPKSSTE;
	int markerPKPSTE;

	string airfoilGeoName;
	string airfoilGeoNameShort;

	vector<vector<double>> xyCoordinateList;
	vector<Node> nodes;
	vector<Node> camberNodes;

	double getRelativeThickness() { return relativeThickness; };
	double getTurbulenceLengthScale() { return chord * 0.07; };
	double getBladeRadius() { return bladeRadius; };
	double getTwist() { return twist; };
	double getChord() { return chord; };
	double getTrailingEdgeSolidAnglePsi() { return trailingEdgeSolidAnglePsi; };
	double getCorrectedTrailingEdgeSolidAnglePsi() { return correctedTrailingEdgeSolidAnglePsi; };
	double getCorrectedTrailingEdgeThickness() { return correctedTrailingEdgeThickness; };

	void setRelativeThickness(double value) { relativeThickness = value; };
	void setBladeRadius(double value) { bladeRadius = value; };
	void setTwist(double value) { twist = value; };
	void setChord(double value) { chord = value; };

	double getTrailingEdgeThickness() { return sqrt(pow(nodes.at(markerTESSMAX).Y() * chord, 2) + pow(nodes.at(markerTEPSMAX).Y() * chord, 2)); };
	double getTrailingEdgeThicknessNew() { return calcNormalizedThickness(1.0) * chord; };

	double getTrailingEdgeStreamAnglePsiAtSS();
	double getTrailingEdgeStreamAnglePsiAtPS();
	double getTrailingEdgeSolidAnglePsiNew(class ProjectData* pPD);
	vector<double> getPolynomialFit(vector<double> x_ss, vector<double> y_ss);
	double calcNormalizedThickness(double relativePositionOnChord);
	void calcCorrectedTrailingEdgeThickness();
	void calcAirfoilCamberLine();

private:
	double relativeThickness;
	double bladeRadius;
	double twist;
	double chord;
	double trailingEdgeSolidAnglePsi;
	double correctedTrailingEdgeSolidAnglePsi;
	double correctedTrailingEdgeThickness;

};
