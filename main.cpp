#include "main.h"
#include <iostream>
#include <sstream>
#include <direct.h>
#include <Windows.h> 
// #include <filesystem>

#include <string>
#include <exception>
#include <fstream>
#include <algorithm>
#include <iomanip>

#include "CubicSpline.h"
#include "spline.h"
#include "Node.h"

#define PI 3.14159265

using namespace std;

int main(int argc, char* argv[])
{
	string projectDataFileName("Project.ini");

	if (argc > 1) {
		projectDataFileName.clear();
		projectDataFileName.assign(argv[1]);
	}
	string logName(toolName + ".log");
	string logMessage = "";
	pLogFile = new ofstream(logName);

	ProjectData* pPD = new ProjectData();

	readProjectFile(projectDataFileName, pPD);

	writeLogFile(*pLogFile, "Done reading Project\n");

	AirfoilGeoSummaryData* pAGSD = new AirfoilGeoSummaryData();

	readAirfoilGeoSummaryFile(pPD->airfoilGeoSummaryFileName, pAGSD);

	readAirfoilGeoFiles(pAGSD);

	BladeGeoData* pBGD = new BladeGeoData();

	readBladeGeoFile(pPD->bladeGeoFileName, pBGD);

	interpolateAirfoilGeoData(pBGD, pAGSD, pPD);

	writeAirfoilGeoInterpolatedFiles(pBGD, pAGSD);

	writeAirfoilMetaData(pAGSD, pPD);

	writeAirfoilCamberLineData(pAGSD);

	pLogFile->close();

}

void writeLogFile(ofstream& outputLogFile, string logMessage)
{
	outputLogFile.write(logMessage.c_str(), logMessage.size());
}

void trimFlatbackPoints(class AirfoilGeoData* pAGD)
{
	// clean airfoil input data -> erase flatback points
	int topFlatbackPoint, bottomFlatbackPoint;
	for (Node& element : pAGD->nodes)
	{
		if (element.IsLastFlatbackPoint() && element.IsTop())
		{
			topFlatbackPoint = element.Index();
		}
		if (element.IsLastFlatbackPoint() && (element.IsTop() == false))
		{
			bottomFlatbackPoint = element.Index();
			break;
		}
	}
	size_t trimFromEnd = pAGD->nodes.size() - bottomFlatbackPoint - 1;
	size_t trimFromBeginning = topFlatbackPoint;
	pAGD->nodes.erase(pAGD->nodes.begin(), pAGD->nodes.begin() + trimFromBeginning);
	pAGD->nodes.erase(pAGD->nodes.end() - trimFromEnd, pAGD->nodes.end());
}

void interpolateAirfoilGeoData(class BladeGeoData* pBGD, class AirfoilGeoSummaryData* pAGSD, class ProjectData* pPD) {

	//Task 0: interpolate rel thicknesses for provided sections
	vector<double> relDv = pBGD->bladeRelativeThickness;
	vector<double> vBladeRadius = pBGD->bladeRadius;


	assignBladeAnalyseSections(pPD);

	for (int i = 0; i < pPD->bladeAnalyseSections.size(); i++)
	{
		double radiusTarget = pPD->bladeAnalyseSections.at(i);
		double relThicknessTarget = interpol(vBladeRadius, relDv, radiusTarget);
		pPD->relThicknessAtAnalyseSections.push_back(relThicknessTarget);
	}

	//Task1: loop over all rel thicknesses to interpolate and find in each loop step the two airfoils to interpolate between
	for (int k = 0; k < pAGSD->AirfoilGeoDataList.size(); k++)
	{
		for (int i = 0; i < (pPD->relThicknessAtAnalyseSections.size()); i++)
		{
			if (pPD->relThicknessAtAnalyseSections.at(i) >= pAGSD->AirfoilGeoDataList.at(k)->getRelativeThickness())
			{
				continue;
			}
			else if (pPD->relThicknessAtAnalyseSections.at(i) > pAGSD->AirfoilGeoDataList.at(k + 1)->getRelativeThickness())
			{
				// Task2: interpolation preparation starts here

				AirfoilGeoData* airfoilThick = NULL;
				AirfoilGeoData* airfoilThin = NULL;
				double relThickThick;
				double relThicknessThin;
				double relThicknessTarget = pPD->relThicknessAtAnalyseSections.at(i);

				if (pAGSD->AirfoilGeoDataList.at(k)->getRelativeThickness() > pAGSD->AirfoilGeoDataList.at(k + 1)->getRelativeThickness())
				{
					airfoilThick = pAGSD->AirfoilGeoDataList.at(k);
					airfoilThin = pAGSD->AirfoilGeoDataList.at(k + 1);
					relThickThick = pAGSD->AirfoilGeoDataList.at(k)->getRelativeThickness();
					relThicknessThin = pAGSD->AirfoilGeoDataList.at(k + 1)->getRelativeThickness();
				}
				else if (pAGSD->AirfoilGeoDataList.at(k)->getRelativeThickness() < pAGSD->AirfoilGeoDataList.at(k + 1)->getRelativeThickness())
				{
					airfoilThick = pAGSD->AirfoilGeoDataList.at(k + 1);
					airfoilThin = pAGSD->AirfoilGeoDataList.at(k);
					relThickThick = pAGSD->AirfoilGeoDataList.at(k + 1)->getRelativeThickness();
					relThicknessThin = pAGSD->AirfoilGeoDataList.at(k)->getRelativeThickness();
				}
				else
				{
					cout << "Airfoils have same relative thickness!" << endl;
				}

				double percent = (relThicknessTarget - relThickThick) / (relThicknessThin - relThickThick);

				pAGSD->AirfoilInterpolatedGeoDataList.push_back(new AirfoilGeoData());
				pAGSD->AirfoilInterpolatedGeoDataList.back()->setRelativeThickness(pPD->relThicknessAtAnalyseSections.at(i));
				pAGSD->AirfoilInterpolatedGeoDataList.back()->setBladeRadius(pPD->bladeAnalyseSections.at(i));

				pAGSD->AirfoilInterpolatedGeoDataList.back()->setTwist(interpol(pBGD->bladeRadius, pBGD->bladeTwist, pPD->bladeAnalyseSections.at(i)));
				pAGSD->AirfoilInterpolatedGeoDataList.back()->setChord(interpol(pBGD->bladeRadius, pBGD->bladeChord, pPD->bladeAnalyseSections.at(i)));

				ostringstream ssRelThickness;
				ostringstream ssBladeSection;
				ostringstream ssInterpolatedAirfoilName;
				ostringstream ssInterpolatedAirfoilNameShort;
				int decimalPlaces = 4;
				double roundedValueRelativeThickness = std::round(pAGSD->AirfoilInterpolatedGeoDataList.back()->getRelativeThickness() * std::pow(10, decimalPlaces)) / std::pow(10, decimalPlaces);
				double roundedAnalyseSection = std::round(pPD->bladeAnalyseSections.at(i) * std::pow(10, decimalPlaces)) / std::pow(10, decimalPlaces);
				ssInterpolatedAirfoilName << "L" << fixed << setprecision(4) << roundedAnalyseSection << "_RT" << fixed << setprecision(4) << roundedValueRelativeThickness;
				ssInterpolatedAirfoilNameShort << "L" << fixed << setprecision(4) << roundedAnalyseSection;
				pAGSD->AirfoilInterpolatedGeoDataList.back()->airfoilGeoName = ssInterpolatedAirfoilName.str(); //LXXX.XXXX_RTXXX.XXXX.geo
				pAGSD->AirfoilInterpolatedGeoDataList.back()->airfoilGeoNameShort = ssInterpolatedAirfoilNameShort.str(); //LXXX.XXXX.geo


				// Task2.1: we need x vals of both airfoil objects in a list
				vector<double> xListTop, xListBottom; // interpolate both airfoil on these lists
				vector<double> xListTopThick, yListTopThick, xListBottomThick, yListBottomThick;
				vector<double> xListTopThin, yListTopThin, xListBottomThin, yListBottomThin;

				vector<double> yListTopThickNew, yListBottomThickNew;
				vector<double> yListTopThinNew, yListBottomThinNew;

				vector<double> yListTopNew, yListBottomNew; // new y lists of interpolated airfoil


				for (Node& element : airfoilThick->nodes)
				{
					if (element.IsTop())
					{
						xListTop.push_back(element.X());

						xListTopThick.push_back(element.X());
						yListTopThick.push_back(element.Y());
					}
					else
					{
						xListBottom.push_back(element.X());

						xListBottomThick.push_back(element.X());
						yListBottomThick.push_back(element.Y());
					}
				}
				for (Node& element : airfoilThin->nodes)
				{
					if (element.IsTop())
					{
						xListTop.push_back(element.X());

						xListTopThin.push_back(element.X());
						yListTopThin.push_back(element.Y());
					}
					else
					{
						xListBottom.push_back(element.X());

						xListBottomThin.push_back(element.X());
						yListBottomThin.push_back(element.Y());
					}
				}

				// get rid of dublicates in list top and bottom
				// top
				sort(xListTop.begin(), xListTop.end());
				auto it = unique(xListTop.begin(), xListTop.end());
				xListTop.erase(it, xListTop.end());

				// bottom
				// insert 0 at start because 0 is part of top when data is read in
				xListBottom.insert(xListBottom.begin(), 0.0);

				xListBottomThick.insert(xListBottomThick.begin(), 0.0);
				xListBottomThin.insert(xListBottomThin.begin(), 0.0);
				yListBottomThick.insert(yListBottomThick.begin(), 0.0);
				yListBottomThin.insert(yListBottomThin.begin(), 0.0);


				sort(xListBottom.begin(), xListBottom.end());
				auto it2 = unique(xListBottom.begin(), xListBottom.end());
				xListBottom.erase(it2, xListBottom.end());

				// Task2.2: we need for the x val list all y val in both airfoils
				// top
				// reverse for interpolation
				reverse(xListTopThick.begin(), xListTopThick.end());
				reverse(yListTopThick.begin(), yListTopThick.end());
				reverse(xListTopThin.begin(), xListTopThin.end());
				reverse(yListTopThin.begin(), yListTopThin.end());
				for (double& element : xListTop)
				{
					yListTopThickNew.push_back(interpol(xListTopThick, yListTopThick, element));
					yListTopThinNew.push_back(interpol(xListTopThin, yListTopThin, element));
				}
				// reverse top again
				reverse(yListTopThickNew.begin(), yListTopThickNew.end());
				reverse(yListTopThinNew.begin(), yListTopThinNew.end());

				for (double& element : xListBottom)
				{
					yListBottomThickNew.push_back(interpol(xListBottomThick, yListBottomThick, element));
					yListBottomThinNew.push_back(interpol(xListBottomThin, yListBottomThin, element));
				}
				// reverse bottom again
				// Task2.3 now we can linear interpolate between both airfoils because we have in both the same x vals
				// top
				int indexCounter = 0;
				vector<double> xListTop_reversed = xListTop;
				reverse(xListTop_reversed.begin(), xListTop_reversed.end());

				// put first poin 1 , 0
				pAGSD->AirfoilInterpolatedGeoDataList.back()->nodes.emplace_back(indexCounter, 1.0, 0.0, true, false);

				for (size_t i = 0; i < xListTop.size(); ++i) {
					yListTopNew.push_back((1.0 - percent) * yListTopThickNew[i] + percent * yListTopThinNew[i]);
					pAGSD->AirfoilInterpolatedGeoDataList.back()->nodes.emplace_back(indexCounter, xListTop_reversed[i], yListTopNew[i], true, false);
					indexCounter++;
				}
				// flip top vectors because we start at TE x=1

				// bottom
				// remove 0,0 values because we get this point drom top side
				xListBottom.erase(xListBottom.begin());
				yListBottomThinNew.erase(yListBottomThinNew.begin());
				yListBottomThickNew.erase(yListBottomThickNew.begin());

				for (size_t i = 0; i < xListBottom.size(); ++i) {
					yListBottomNew.push_back((1.0 - percent) * yListBottomThickNew[i] + percent * yListBottomThinNew[i]);
					pAGSD->AirfoilInterpolatedGeoDataList.back()->nodes.emplace_back(indexCounter, xListBottom[i], yListBottomNew[i], false, false);
					indexCounter++;
				}
				// put last point 1 , 0
				pAGSD->AirfoilInterpolatedGeoDataList.back()->nodes.emplace_back(indexCounter, 1.0, 0.0, false, true);
				pAGSD->AirfoilInterpolatedGeoDataList.back()->calcAirfoilCamberLine();
			}
		}
	}
}

void assignAirfoilMarker(class AirfoilGeoSummaryData* pAGSD)
{
	for (int k = 0; k < pAGSD->AirfoilInterpolatedGeoDataList.size(); k++)
	{
		for (int i = 0; i < pAGSD->AirfoilInterpolatedGeoDataList.at(k)->nodes.size(); i++)
		{
			pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerTEE = 1;

			if ((pAGSD->AirfoilInterpolatedGeoDataList.at(k)->nodes.at(i).X() == 0.0) && (pAGSD->AirfoilInterpolatedGeoDataList.at(k)->nodes.at(i).Y() == 0.0))
			{
				pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerLE = i + 1;
				pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerLEPSMAX = i + 1 + 20;
				pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerLESSMAX = i + 1 - 20;
			}

			pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerTE = pAGSD->AirfoilInterpolatedGeoDataList.at(k)->nodes.size();
			pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerTEPSMAX = pAGSD->AirfoilInterpolatedGeoDataList.at(k)->nodes.size() - 1;
			pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerTEPS = pAGSD->AirfoilInterpolatedGeoDataList.at(k)->nodes.size() - 1;
			pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerTESSMAX = 2;
			pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerTESS = 2;
		}
	}
}

void assignBladeAnalyseSections(ProjectData* pPD)
{
	if (!pPD->takeAnalyseSectionsInput)
	{
		getBladeAnalyseSections(pPD);
	}
}

void getBladeAnalyseSections(ProjectData* pPD)
{
	double mainResolution = 1;
	double lscale = 1.0 / (pPD->hubRadius + pPD->bladeLength);
	vector<double> bladeTipSegmentWidth = { 0.1, 0.1, 0.1, 0.2, 0.2, 0.3, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9 };

	pPD->bladeAnalyseSections.push_back(pPD->bladeLength - bladeTipSegmentWidth[0] / 2.0);
	double r0 = 1.0 / 3.0;
	for (int i = 0; i < bladeTipSegmentWidth.size() - 1; i++)
	{
		pPD->bladeAnalyseSections.push_back(pPD->bladeAnalyseSections.back() - (bladeTipSegmentWidth.at(i) + bladeTipSegmentWidth.at(i + 1)) / 2.0);
	}
	while (pPD->bladeAnalyseSections.back() > (r0 / lscale - pPD->hubRadius))
	{
		pPD->bladeAnalyseSections.push_back(pPD->bladeAnalyseSections.back() - (bladeTipSegmentWidth.back() + mainResolution) / 2.0);
		bladeTipSegmentWidth.push_back(mainResolution);
	}

	reverse(pPD->bladeAnalyseSections.begin(), pPD->bladeAnalyseSections.end());
	reverse(bladeTipSegmentWidth.begin(), bladeTipSegmentWidth.end());
	pPD->bladeSegmentWidth = bladeTipSegmentWidth;

	vector<double> relative_blade_analyse_sections;
	for (const auto& element : pPD->bladeAnalyseSections) {
		relative_blade_analyse_sections.push_back((element + pPD->hubRadius) * lscale);
	}

}

void writeAirfoilMetaData(class AirfoilGeoSummaryData* pAGSD, ProjectData* pPD)
{
	string AirfoilDir = "Airfoils";
	AirfoilNoiseDir = "Airfoils\\BladeNoise";
	string fileName = "AirfoilMetaData";
	ofstream outputFile(AirfoilNoiseDir + "\\" + fileName + ".dat");

	outputFile << "Airfoil File Name [-]\t";
	outputFile << "Blade Radius [m]\t";
	outputFile << "Chord [m]\t";
	outputFile << "Twist [deg]\t";
	outputFile << "Relative Thickness [%]\t";
	outputFile << "Trailing Edge Solid Angle Psi [deg]" << "\t";
	outputFile << "Trailing Edge Thickness [m]\t";
	outputFile << "Turbulence Length Scale [m]" << "\t";
	outputFile << "Relative Thickness At 1 % Chord [-]" << "\t";
	outputFile << "Relative Thickness At 10 % Chord [-]" << "\t";
	outputFile << "Airfoil Analyse Segment Width [m]" << "\n";

	for (int k = 0; k < pAGSD->AirfoilInterpolatedGeoDataList.size(); k++)
	{
		outputFile << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->airfoilGeoNameShort << "\t";
		outputFile << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->getBladeRadius() << "\t";
		outputFile << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->getChord() << "\t";
		outputFile << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->getTwist() << "\t";
		outputFile << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->getRelativeThickness() << "\t";
		outputFile << (pAGSD->AirfoilInterpolatedGeoDataList.at(k)->getTrailingEdgeSolidAnglePsiNew(pPD)) << "\t";
		outputFile << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->getCorrectedTrailingEdgeThickness() << "\t";
		outputFile << (pAGSD->AirfoilInterpolatedGeoDataList.at(k)->getTurbulenceLengthScale()) << "\t";
		outputFile << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->calcNormalizedThickness(0.01) << "\t";
		outputFile << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->calcNormalizedThickness(0.1) << "\t";
		outputFile << pPD->bladeSegmentWidth.at(k) << "\n";
	}
	outputFile.close();

	fileName = "GeometrieDat4AirfoilMetaData";
	ofstream outputFile2(AirfoilNoiseDir + "\\" + fileName + ".dat");

	outputFile2 << "#	Revision	" << "\n";
	outputFile2 << "#	Last Export :	" << "\n";
	outputFile2 << "#	#Radius ab Wurzel	Tiefe	Bauwinkel	Relative Dicke	x-Lage t/4	y-Lage t/4	PCBA-X	PCBA-Y	Rel. Twistachse" << "\n";
	outputFile2 << "#	#[m]	[m]	[deg]	[%]	[m]	[m]	[m]	[m]	[%]";
	for (int k = 0; k < pAGSD->AirfoilInterpolatedGeoDataList.size(); k++)
	{
		outputFile2 << "\nDEF\t";
		outputFile2 << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->getBladeRadius() << "\t";
		outputFile2 << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->getChord() << "\t";
		outputFile2 << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->getTwist() << "\t";
		outputFile2 << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->getRelativeThickness() << "\t";
		outputFile2 << "0\t";
		outputFile2 << "0\t";
		outputFile2 << "0\t";
		outputFile2 << "0\t";
		outputFile2 << "0";
	}
	outputFile2.close();

	writeLogFile(*pLogFile, "Done writing airfoil meta data and corresponding blade geometry table.\n");
}


string ExePath() {
	char buffer[MAX_PATH];
	GetModuleFileName(NULL, (LPWSTR)buffer, MAX_PATH);
	std::string::size_type pos = std::string(buffer).find_last_of("\\/");
	if (pos != std::string::npos) {
		return std::string(buffer).substr(0, pos);
	}
	return "";
}

void writeAirfoilCamberLineData(class AirfoilGeoSummaryData* pAGSD)
{
	string AirfoilDir = "Airfoils";
	string fileNamePrefix = "ACL_";
	for (int k = 0; k < pAGSD->AirfoilInterpolatedGeoDataList.size(); k++)
	{
		string fileName = pAGSD->AirfoilInterpolatedGeoDataList.at(k)->airfoilGeoName;
		ofstream outputFile(AirfoilACLDir + "\\" + fileNamePrefix + fileName + ".geo");
		outputFile << "# Airfoil camber line " << "\n";
		outputFile << "# Name:\t" << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->airfoilGeoName << "\n";
		outputFile << "# X [-]\t Y [-]" << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->airfoilGeoName << "\n";
		for (int i = 0; i < pAGSD->AirfoilInterpolatedGeoDataList.at(k)->camberNodes.size(); i++)
		{
			outputFile << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->camberNodes.at(i).X() << "\t";
			outputFile << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->camberNodes.at(i).Y() << "\n";
		}
		outputFile.close();
	}
}

void writeAirfoilGeoInterpolatedFiles(class BladeGeoData* pBGD, class AirfoilGeoSummaryData* pAGSD) {

	AirfoilDir = "Airfoils";
	AirfoilACLDir = "Airfoils\\ACL";
	AirfoilNoiseDir = "Airfoils\\BladeNoise";

	vector<string> pathList;

	Create_Directory(AirfoilDir);
	Create_Directory(AirfoilACLDir);
	Create_Directory(AirfoilNoiseDir);

	assignAirfoilMarker(pAGSD);

	ofstream outputFileYAML(AirfoilDir + "\\" + "AirfoilList" + ".yaml");
	outputFileYAML << "Coordinates: [";

	for (int k = 0; k < pAGSD->AirfoilInterpolatedGeoDataList.size(); k++) {

		pAGSD->AirfoilInterpolatedGeoDataList.at(k)->airfoilGeoName;
		string fileName = pAGSD->AirfoilInterpolatedGeoDataList.at(k)->airfoilGeoNameShort;
		ofstream outputFile(AirfoilNoiseDir + "\\" + fileName + ".dat");

		outputFile << fileName << "\n";

		for (int i = 0; i < pAGSD->AirfoilInterpolatedGeoDataList.at(k)->nodes.size(); i++)
		{
			if ((i > 0) && (i < pAGSD->AirfoilInterpolatedGeoDataList.at(k)->nodes.size() - 1))
			{
				outputFile << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->nodes.at(i).X() << " " << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->nodes.at(i).Y() << "\n";
			}
		}
		outputFile.close();
	}


	for (int k = 0; k < pAGSD->AirfoilInterpolatedGeoDataList.size(); k++) {

		pAGSD->AirfoilInterpolatedGeoDataList.at(k)->airfoilGeoName;
		string fileName = pAGSD->AirfoilInterpolatedGeoDataList.at(k)->airfoilGeoName;
		double relThickness = pAGSD->AirfoilInterpolatedGeoDataList.at(k)->getRelativeThickness();
		ofstream outputFile(AirfoilDir + "\\" + fileName + ".geo");

		outputFileYAML << "\n" + ExePath() + "\\" + AirfoilDir + "\\" + fileName + ".geo,";

		outputFile << "NAME\t" << fileName << "\n";
		outputFile << "RELDICKE\t" << relThickness << "\n";
		outputFile << "MARKER\tTEE\t" << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerTEE << "\n";
		outputFile << "MARKER\tTESSMAX\t" << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerTESSMAX << "\n";
		outputFile << "MARKER\tTESS\t" << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerTESS << "\n";
		outputFile << "MARKER\tLESSMAX\t" << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerLESSMAX << "\n";
		outputFile << "MARKER\tLE\t" << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerLE << "\n";
		outputFile << "MARKER\tLEPSMAX\t" << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerLEPSMAX << "\n";
		outputFile << "MARKER\tTEPS\t" << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerTEPS << "\n";
		outputFile << "MARKER\tTEPSMAX\t" << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerTEPSMAX << "\n";
		outputFile << "MARKER\tTE\t" << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->markerTE << "\n";
		outputFile << "#\tX\tY" << "\n";

		for (int i = 0; i < pAGSD->AirfoilInterpolatedGeoDataList.at(k)->nodes.size(); i++)
		{
			outputFile << "DEF\t" << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->nodes.at(i).X() << "\t" << pAGSD->AirfoilInterpolatedGeoDataList.at(k)->nodes.at(i).Y() << "\n";
		}
		outputFile.close();
	}
	outputFileYAML << "\n]";
	outputFileYAML << "\nACL_Coordinates: [";
	outputFileYAML << "\n]";
	outputFileYAML.close();
}

void readProjectFile(string fileName, class ProjectData* pPD)
{
	ifstream inputFile(fileName);
	if (!inputFile.is_open())
	{
		cout << "Error opening file!" << endl;
	}
	string line;
	string buf;
	string buffer;
	pPD->takeAnalyseSectionsInput = false;

	while (!inputFile.eof())
	{
		inputFile >> buffer;
		if (!buffer.compare("#"))
		{
			continue;
		}
		if (!buffer.compare("HUB_RADIUS"))
		{
			inputFile >> buffer;
			string hubRadius = buffer;
			pPD->hubRadius = stod(hubRadius);
		}
		if (!buffer.compare("BLADE_LENGTH"))
		{
			inputFile >> buffer;
			string bladeLength = buffer;
			pPD->bladeLength = stod(bladeLength);
		}
		if (!buffer.compare("AIRFOIL_GEO"))
		{
			inputFile >> buffer;
			string airfoilGeoSummaryFileName = buffer;
			pPD->airfoilGeoSummaryFileName = airfoilGeoSummaryFileName;
		}
		if (!buffer.compare("AIRFOIL_PERFO"))
		{
			inputFile >> buffer;
			string airfoilPerfoSummaryFileName = buffer;
			pPD->airfoilPerfoSummaryFileName = airfoilPerfoSummaryFileName;
		}
		if (!buffer.compare("BLADE_GEO"))
		{
			inputFile >> buffer;
			string bladeGeoFileName = buffer;
			pPD->bladeGeoFileName = bladeGeoFileName;
		}
		if (!buffer.compare("ANALYSE_SECTIONS"))
		{
			pPD->takeAnalyseSectionsInput = true;
			getline(inputFile, buffer);
			replace(buffer.begin(), buffer.end(), '[', ' ');
			replace(buffer.begin(), buffer.end(), ']', ' ');
			replace(buffer.begin(), buffer.end(), ',', ' ');
			size_t pos = buffer.find('#');
			buffer = buffer.substr(0, pos); // Remove all characters after the given position 
			stringstream ssBuffer(buffer);
			string i;
			while (ssBuffer >> i)
			{
				pPD->bladeAnalyseSections.push_back(stod(i));
			}
		}
	}
	inputFile.close();
}

void readAirfoilGeoSummaryFile(string fileName, class AirfoilGeoSummaryData* pAGSD)
{
	ifstream inputFile(fileName);
	if (!inputFile.is_open())
	{
		cout << "Error opening file!" << endl;
	}
	string line;
	string buffer;
	while (!inputFile.eof())
	{
		inputFile >> buffer;
		if (!buffer.compare("#"))
		{
			continue;
		}
		if (!buffer.compare("DEF"))
		{
			inputFile >> buffer;
			if (buffer == "[m]")
			{
				continue;
			}
			inputFile >> buffer >> buffer;
			string airfoilGeoDataFileName = buffer;
			pAGSD->AirfoilGeoDataFileNameList.push_back(airfoilGeoDataFileName);
		}
	}
	inputFile.close();
}

void readAirfoilGeoFiles(class AirfoilGeoSummaryData* pAGSD)
{

	for (int i = 0; i < pAGSD->AirfoilGeoDataFileNameList.size(); i++)
	{

		pAGSD->AirfoilGeoDataList.push_back(new AirfoilGeoData()); //.push_back(new AirfoilGeoData());

		ifstream inputFile(pAGSD->AirfoilGeoDataFileNameList.at(i));
		if (!inputFile.is_open())
		{
			cout << "Error opening file!" << endl;
		}
		string line;
		string buffer, buffer2, buffer3;
		int indexCounter = 0;
		double xLast, xNext;

		while (!inputFile.eof())
		{
			string line = "";
			inputFile >> buffer;
			if (!buffer.compare("NAME"))
			{
				string airfoilGeoName;
				inputFile >> airfoilGeoName;
				pAGSD->AirfoilGeoDataList.at(i)->airfoilGeoName = airfoilGeoName;
			}
			if (!buffer.compare("RELDICKE"))
			{
				string relativeThickness;
				inputFile >> relativeThickness;
				pAGSD->AirfoilGeoDataList.at(i)->setRelativeThickness(stod(relativeThickness)); 
			}
			if (!buffer.compare("MARKER"))
			{
				inputFile >> buffer;
				if (!buffer.compare("TEE"))
				{
					string TEE;
					inputFile >> TEE;
					pAGSD->AirfoilGeoDataList.at(i)->markerTEE = stoi(TEE);
				}
				if (!buffer.compare("TESSMAX"))
				{
					string TESSMAX;
					inputFile >> TESSMAX;
					pAGSD->AirfoilGeoDataList.at(i)->markerTESSMAX = stoi(TESSMAX);
				}
				if (!buffer.compare("TESS"))
				{
					string TESS;
					inputFile >> TESS;
					pAGSD->AirfoilGeoDataList.at(i)->markerTESS = stoi(TESS);
				}
				if (!buffer.compare("PKSSTE"))
				{
					string PKSSTE;
					inputFile >> PKSSTE;
					pAGSD->AirfoilGeoDataList.at(i)->markerPKSSTE = stoi(PKSSTE);
				}
				if (!buffer.compare("LESSMAX"))
				{
					string LESSMAX;
					inputFile >> LESSMAX;
					pAGSD->AirfoilGeoDataList.at(i)->markerLESSMAX = stoi(LESSMAX);
				}
				if (!buffer.compare("LE"))
				{
					string LE;
					inputFile >> LE;
					pAGSD->AirfoilGeoDataList.at(i)->markerLE = stoi(LE);
				}
				if (!buffer.compare("LEPSMAX"))
				{
					string LEPSMAX;
					inputFile >> LEPSMAX;
					pAGSD->AirfoilGeoDataList.at(i)->markerLEPSMAX = stoi(LEPSMAX);
				}
				if (!buffer.compare("PKPSTE"))
				{
					string PKPSTE;
					inputFile >> PKPSTE;
					pAGSD->AirfoilGeoDataList.at(i)->markerPKPSTE = stoi(PKPSTE);
				}
				if (!buffer.compare("TEPS"))
				{
					string TEPS;
					inputFile >> TEPS;
					pAGSD->AirfoilGeoDataList.at(i)->markerTEPS = stoi(TEPS);
				}
				if (!buffer.compare("TEPSMAX"))
				{
					string TEPSMAX;
					inputFile >> TEPSMAX;
					pAGSD->AirfoilGeoDataList.at(i)->markerTEPSMAX = stoi(TEPSMAX);
				}
				if (!buffer.compare("TE"))
				{
					string TE;
					inputFile >> TE;
					pAGSD->AirfoilGeoDataList.at(i)->markerTE = stoi(TE);
				}
			}
			if (!buffer.compare("DEF"))
			{
				string X;
				string Y;
				bool isTop = true;
				bool isLastFlatbackPoint = false;
				inputFile >> X;
				inputFile >> Y;
				if (X == "")
				{
					continue;
				}
				double x = stod(X);
				xNext = x;
				double y = stod(Y);
				if (indexCounter > 0)
				{

					xLast = pAGSD->AirfoilGeoDataList.at(i)->nodes.back().X();
					if (xNext == xLast)
					{
						isLastFlatbackPoint = true;
					}
					if (xNext == 1 && xLast != 1)
					{
						isLastFlatbackPoint = true;
					}
					if (xLast >= xNext)
					{
						isTop = true;
					}
					else
					{
						isTop = false;
					}
				}
				else
				{
					xLast = x;
					isLastFlatbackPoint = true;
				}
				pAGSD->AirfoilGeoDataList.at(i)->nodes.emplace_back(indexCounter, x, y, isTop, isLastFlatbackPoint);
				indexCounter++;
			}
		}
		inputFile.close();
		trimFlatbackPoints(pAGSD->AirfoilGeoDataList.at(i));
	}
}

void removeCharsFromString(std::string& str, const char* charsToRemove) {
	for (unsigned int i = 0; i < std::strlen(charsToRemove); ++i) {
		str.erase(std::remove(str.begin(), str.end(), charsToRemove[i]), str.end());
	}
}

void readBladeGeoFile(string fileName, class BladeGeoData* pBGD)
{
	ifstream inputFile(fileName);
	if (!inputFile.is_open())
	{
		cout << "Error opening file!" << endl;
	}
	string buffer;
	while (!inputFile.eof())
	{
		inputFile >> buffer;
		if (!buffer.compare("#"))
		{
			continue;
		}
		if (!buffer.compare("DEF"))
		{
			inputFile >> buffer;
			double radius = stod(buffer);
			pBGD->bladeRadius.push_back(radius);
			inputFile >> buffer;
			double chord = stod(buffer);
			pBGD->bladeChord.push_back(chord);
			inputFile >> buffer;
			double twist = stod(buffer);
			pBGD->bladeTwist.push_back(twist);
			inputFile >> buffer;
			double relThickness = stod(buffer);
			pBGD->bladeRelativeThickness.push_back(relThickness);
			inputFile >> buffer;
			double xPosT4 = stod(buffer);
			pBGD->bladeXt4Position.push_back(xPosT4);
			inputFile >> buffer;
			double yPosT4 = stod(buffer);
			pBGD->bladeYt4Position.push_back(yPosT4);
			inputFile >> buffer;
			double PCBAX = stod(buffer);
			pBGD->bladePCBAX.push_back(PCBAX);
			inputFile >> buffer;
			double PCBAY = stod(buffer);
			pBGD->bladePCBAY.push_back(PCBAY);
			inputFile >> buffer;
			double relTwistAxis = stod(buffer);
			pBGD->bladeRelativeTwistAxis.push_back(relTwistAxis);
		}
	}
	inputFile.close();
}

double interpol(vector<double> x, vector<double> y, double xtarget)
{
	if (y.size() != x.size()) cout << "Error: x and y arrays have not the same size" << endl;
	size_t i;
	if (xtarget < x.at(0)) return y.at(0);		// Wenn xtarget unten out of range, dann untersten y-wert zuruck
	if (xtarget > x.back()) return y.back();	// Wenn xtarget oben out of range, dann obersten y-wert zuruck
	for (i = 0; i < x.size() - 1; i++)
		if ((x.at(i) <= xtarget) && x.at(i + 1) >= xtarget) break;
	return y.at(i) + (xtarget - x.at(i)) * (y.at(i + 1) - y.at(i)) / (x.at(i + 1) - x.at(i));
}

void Create_Directory(string sVerzeichnis)
{
	string sVerzeichnisArbeit = "\\..";

	if (_chdir(sVerzeichnis.c_str()) != 0) {
		cout << "Directory " << sVerzeichnis << " created!" << endl;
		_mkdir(sVerzeichnis.c_str());
	}
	else _chdir(sVerzeichnisArbeit.c_str());
}