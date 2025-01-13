#define _CRT_SECURE_NO_DEPRECATE 1
#define _CRT_NONSTDC_NO_DEPRECATE 1

#include <stdarg.h>
#include <string>
#include <iostream>
#include <vector>

#include "ProjectData.h"
#include "AirfoilGeoData.h"
#include "AirfoilGeoSummaryData.h"
#include "BladeGeoData.h"

using namespace std;

ofstream* pLogFile;
string toolName = "AirfoilProcessing";
string AirfoilDir;
string AirfoilACLDir;
string AirfoilNoiseDir;

void readProjectFile(string fileName, class ProjectData* pPD);

void readAirfoilGeoSummaryFile(string fileName, class AirfoilGeoSummaryData* pAGSD);

void readAirfoilGeoFiles(class AirfoilGeoSummaryData* pAGSD);

void readBladeGeoFile(string fileName, class BladeGeoData* pBGD);

void writeAirfoilGeoInterpolatedFiles(class BladeGeoData* pBGD, class AirfoilGeoSummaryData* pAGSD);

void interpolateAirfoilGeoData(class BladeGeoData* pBGD, class AirfoilGeoSummaryData* pAGSD, class ProjectData* pPD);

double interpol(vector<double> x, vector<double> y, double xtarget);

void trimFlatbackPoints(class AirfoilGeoData* pAGD);

void Create_Directory(string sVerzeichnis);

void assignAirfoilMarker(class AirfoilGeoSummaryData* pAGSD);

void getBladeAnalyseSections(ProjectData* pPD);

void assignBladeAnalyseSections(ProjectData* pPD);

void writeAirfoilMetaData(class AirfoilGeoSummaryData* pAGSD, ProjectData* pPD);

void writeAirfoilCamberLineData(class AirfoilGeoSummaryData* pAGSD);

void writeLogFile(ofstream& outputLogFile, string logMessage);
