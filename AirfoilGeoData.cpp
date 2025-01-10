#include "AirfoilGeoData.h"
#include "CubicSpline.h"
#include "spline.h"


#define PI 3.14159265

double AirfoilGeoData::getTrailingEdgeStreamAnglePsiAtSS()
{
	vector<double> x_ss, y_ss;
	x_ss.push_back(nodes.at(1).X());
	x_ss.push_back(nodes.at(2).X());
	x_ss.push_back(nodes.at(3).X());

	y_ss.push_back(nodes.at(1).Y());
	y_ss.push_back(nodes.at(2).Y());
	y_ss.push_back(nodes.at(3).Y());
	//cout << "SS: ";
	vector<double> a = getPolynomialFit(x_ss, y_ss);

	//double S1 = sqrt(pow(nodes.at(2).Y(), 2) - pow(nodes.at(1).Y(), 2));
	//double S2 = sqrt(pow(nodes.at(1).X(), 2) - pow(nodes.at(2).X(), 2));
	//double psiSS = atan(sqrt(pow(nodes.at(2).Y(), 2) - pow(nodes.at(1).Y(), 2)) / sqrt(pow(nodes.at(1).X(), 2) - pow(nodes.at(2).X(), 2))) * 180 / PI;

	double psiSS = atan(a.at(0));

	return psiSS;
}

void AirfoilGeoData::calcAirfoilCamberLine()
{
	vector<double> x_sample, y_t, x_b, y_b, y_b_new;
	for (int i = 1; i < nodes.size(); i++)
	{
		if (nodes.at(i).IsTop() == true)
		{
			y_t.emplace_back(nodes.at(i).Y());
			x_sample.emplace_back(nodes.at(i).X());
		}

	}
	reverse(y_t.begin(), y_t.end());
	reverse(x_sample.begin(), x_sample.end());


	for (size_t i = nodes.size() - 2; i > 0; i--)
	{


		if (nodes.at(i).X() == 0)
		{
			x_b.emplace_back(nodes.at(i).X());
			y_b.emplace_back(nodes.at(i).Y());
			break;
		}
		else
		{
			x_b.emplace_back(nodes.at(i).X());
			y_b.emplace_back(nodes.at(i).Y());

			//double y_c = (nodes.at(i).Y() - nodes.at(nodes.size() - 1 - i).Y()) / 2.0;
			//camberNodes.emplace_back(i,nodes.at(i).X(), y_c, false,false);
		}
	}

	reverse(x_b.begin(), x_b.end());
	reverse(y_b.begin(), y_b.end());



	CubicSpline<double> my_data(x_b, y_b, 2); // the 2 says omp_set_num_threads(2)
	y_b_new = my_data.interpolate(x_sample);

	vector<double> x_only_3_last_points;
	vector<double> y_only_3_last_points;

	for (int i = 0; i < x_sample.size(); i++)
	{
		double y_c = (abs(y_t.at(i)) - abs(y_b_new.at(i))) / 2.0;
		camberNodes.emplace_back(i, x_sample.at(i), y_c, false, false);
	}

	for (size_t i = x_sample.size() - 4; i < x_sample.size(); i++)
	{
		x_only_3_last_points.push_back(x_sample.at(i));
		y_only_3_last_points.push_back(y_b_new.at(i));
	}

	vector<double> a = getPolynomialFit(x_only_3_last_points, y_only_3_last_points);

	trailingEdgeSolidAnglePsi = atan(a.at(1)) * 180 / PI;

	//cout << "ende";
	//x_c = x;
	//y_c = (y_t + y_b) / 2;
}

double AirfoilGeoData::getTrailingEdgeStreamAnglePsiAtPS()
{
	vector<double> x_ps, y_ps;
	x_ps.push_back(nodes.at(nodes.size() - 4).X());
	x_ps.push_back(nodes.at(nodes.size() - 3).X());
	x_ps.push_back(nodes.at(nodes.size() - 2).X());

	y_ps.push_back(nodes.at(nodes.size() - 4).Y());
	y_ps.push_back(nodes.at(nodes.size() - 3).Y());
	y_ps.push_back(nodes.at(nodes.size() - 2).Y());
	//cout << "PS: ";
	vector<double> a = getPolynomialFit(x_ps, y_ps);

	//double S1 = sqrt(pow(nodes.at(nodes.size() - 2).Y(),2) - pow(nodes.at(nodes.size() - 3).Y(),2));
	//double S2 = sqrt(pow(nodes.at(nodes.size() - 2).X(),2) - pow(nodes.at(nodes.size() - 3).X(),2));
	//double psiPS = atan(sqrt(pow(nodes.at(nodes.size()-2).Y(),2) - pow(nodes.at(nodes.size() - 3).Y(),2)) / sqrt(pow(nodes.at(nodes.size() - 2).X(),2) - pow(nodes.at(nodes.size() - 3).X(),2))) * 180 / PI;

	double psiPS = atan(a.at(0));

	return psiPS;
}

double AirfoilGeoData::getTrailingEdgeSolidAnglePsiNew(class ProjectData* pPD)
{
	double solidAngleFactorAtTip = 2.0;
	double solidAngleReduction = 0.2;
	double thicknessReduction = 0.5;
	double minimumMouldGap = 0.001;
	//double trailingEdgeThickness_s;
	double RTESS = 0.005;
	double RTEPS = 0.005;
	double psiPS, psiSS, psiPS_t, psiSS_t, psi; /* , psi_s;*/
	psiSS = getTrailingEdgeStreamAnglePsiAtSS();
	psiPS = getTrailingEdgeStreamAnglePsiAtPS();

	psi = (psiPS - psiSS) * 180 / PI;

	psiSS_t = PI / 2.0 - psiSS;
	psiPS_t = PI / 2.0 + psiPS;

	//calcNormalizedThickness(1.0) * chord

	correctedTrailingEdgeThickness = (getTrailingEdgeThicknessNew() - thicknessReduction * (RTESS / tan(psiSS_t / 2.0) + RTEPS / tan(psiPS_t / 2.0)));

	if (correctedTrailingEdgeThickness < minimumMouldGap)
	{
		correctedTrailingEdgeThickness = minimumMouldGap;
	}

	double correctionPsi = solidAngleReduction * (180 / PI * (2.0 * PI - psiSS_t - psiPS_t) * (1 - correctedTrailingEdgeThickness / sqrt(getTrailingEdgeThicknessNew())) * (1 - sqrt(minimumMouldGap / getTrailingEdgeThicknessNew())));

	//return  (sqrt(pow((psiPS + psiSS), 2))) * 180 / PI;

	if ((correctedTrailingEdgeThickness / getTrailingEdgeThicknessNew()) < 1)
	{
		correctedTrailingEdgeSolidAnglePsi = psi + correctionPsi;

		if ((correctedTrailingEdgeSolidAnglePsi < psi * solidAngleFactorAtTip) && (bladeRadius > 0.9 * pPD->bladeLength))
		{
			correctedTrailingEdgeSolidAnglePsi = psi * solidAngleFactorAtTip;
		}
	}
	else
	{
		correctedTrailingEdgeSolidAnglePsi = psi * solidAngleFactorAtTip;
	}


	//if ((correctedTrailingEdgeSolidAnglePsi < psi * solidAngleFactorAtTip) && (bladeRadius > 0.9 * pPD->bladeLength))
	//{
	//	correctedTrailingEdgeSolidAnglePsi = psi * solidAngleFactorAtTip;
	//}

	return  correctedTrailingEdgeSolidAnglePsi;
}

//void AirfoilGeoData::getTrailingEdgeThicknessNew()
//{
//	double thicknessReduction = 0.5;
//	double minimumMouldGap = 0.001;
//	double RTESS = 0.005;
//	double RTEPS = 0.005;
//
//	trailingEdgeThickness = calcNormalizedThickness(1.0) * chord;
//	if ()
//	{
//
//	}
//
//	return calcNormalizedThickness(1.0) * chord;
//
//	//-IN.thickness_reduction * (RTESS[-1] / np.tan(phiSS / 2.0) + RTEPS[-1] / np.tan(phiPS / 2.0))
//	//double coirrection = ;
//	//double correctedTEThickness = calcNormalizedThickness(1.0) * chord * correction;
//
//	//correctedTrailingEdgeThickness = (getTrailingEdgeThicknessNew() - thicknessReduction * (RTESS / tan(psiSS_t / 2.0) + RTEPS / tan(psiPS_t / 2.0)));
//
//}

double AirfoilGeoData::calcNormalizedThickness(double relativePositionOnChord)
{
	vector<Node> psNodes;
	vector<Node> ssNodes;
	double normalizedThicknessOnChord;
	// we skip first (1 , 0) and last (1 , 0) point here
	for (int i = 1; i < nodes.size() - 1; i++)
	{
		if (nodes.at(i).IsTop())
		{
			ssNodes.push_back(nodes.at(i));
		}
		else
		{
			psNodes.push_back(nodes.at(i));
		}
	}

	psNodes.insert(psNodes.begin(), ssNodes.back());

	// flip ss vector for interpolation
	reverse(ssNodes.begin(), ssNodes.end());

	// reverse ps vector we also need point (0 , 0) 

	vector<double> x_ps, x_ss, y_ss, y_ps;
	for (int i = 0; i < psNodes.size(); i++)
	{
		x_ps.push_back(psNodes.at(i).X());
		y_ps.push_back(psNodes.at(i).Y());
	}
	for (int i = 0; i < ssNodes.size(); i++)
	{
		x_ss.push_back(ssNodes.at(i).X());
		y_ss.push_back(ssNodes.at(i).Y());
	}

	double y_ss_i, y_ps_i;

	//tk::spline s(x_ss, y_ss, tk::spline::cspline); // or tk::spline s(X,Y);
	// alternatively
	tk::spline s_ss, s_ps;
	s_ss.set_points(x_ss, y_ss, tk::spline::cspline); // or s2.set_points(X,Y);
	s_ps.set_points(x_ps, y_ps, tk::spline::cspline); // or s2.set_points(X,Y);

	y_ss_i = s_ss(relativePositionOnChord);
	y_ps_i = s_ps(relativePositionOnChord);


	//y_ss_i = interpol(x_ss, y_ss, relativePositionOnChord);
	//y_ps_i = interpol(x_ps, y_ps, relativePositionOnChord);

	normalizedThicknessOnChord = y_ss_i - y_ps_i;

	return normalizedThicknessOnChord;
}

vector<double> AirfoilGeoData::getPolynomialFit(vector<double> x, vector<double> y)
{
	//vector<double> x_ss, x_ps,y_ss,y_ps;
	//// SS
	//x_ss.push_back(nodes.at(1).X());
	//x_ss.push_back(nodes.at(2).X());
	//x_ss.push_back(nodes.at(3).X());

	//y_ss.push_back(nodes.at(1).Y());
	//y_ss.push_back(nodes.at(2).Y());
	//y_ss.push_back(nodes.at(3).Y());

	////PS
	//x_ps.push_back(nodes.at(nodes.size() - 4).X());
	//x_ps.push_back(nodes.at(nodes.size() - 3).X());
	//x_ps.push_back(nodes.at(nodes.size() - 2).X());
	//  
	//y_ps.push_back(nodes.at(nodes.size() - 4).Y());
	//y_ps.push_back(nodes.at(nodes.size() - 3).Y());
	//y_ps.push_back(nodes.at(nodes.size() - 2).Y());


	int i, j, k;
	int N = 3;
	int n = 1;
	//cout.precision(4);                        //set precision
	//cout.setf(ios::fixed);
	//cout << "\nEnter the no. of data pairs to be entered:\n";        //To find the size of arrays that will store x,y, and z values
	//cin >> N;
	//const double x[N], y[N];
	//cout << "\nEnter the x-axis values:\n";                //Input x-values
	//for (i = 0; i < N; i++)
	//	cin >> x[i];
	//cout << "\nEnter the y-axis values:\n";                //Input y-values
	//for (i = 0; i < N; i++)
	//	cin >> y[i];
	//cout << "\nWhat degree of Polynomial do you want to use for the fit?\n";
	//cin >> n;                                // n is the degree of Polynomial 
	//double X[2 * n + 1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	vector<double> X(2 * n + 1, 0.0);
	for (i = 0; i < 2 * n + 1; i++)
	{
		X[i] = 0;
		for (j = 0; j < N; j++)
			X[i] = X[i] + pow(x[j], i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	}
	//double B[n + 1][n + 2], a[n + 1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
	vector<vector<double>> B(n + 1, vector<double>(n + 2, 0.0));
	vector<double> a(n + 1, 0.0);
	for (i = 0; i <= n; i++)
		for (j = 0; j <= n; j++)
			B[i][j] = X[i + j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
	//double Y[n + 1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	vector<double> Y(n + 1, 0.0);
	for (i = 0; i < n + 1; i++)
	{
		Y[i] = 0;
		for (j = 0; j < N; j++)
			Y[i] = Y[i] + pow(x[j], i) * y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	}
	for (i = 0; i <= n; i++)
		B[i][n + 1] = Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
	n = n + 1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
	//cout << "\nThe Normal(Augmented Matrix) is as follows:\n";
	//for (i = 0; i < n; i++)            //print the Normal-augmented matrix
	//{
	//	for (j = 0; j <= n; j++)
	//		cout << B[i][j] << setw(16);
	//	//cout << "\n";
	//}
	for (i = 0; i < n; i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
		for (k = i + 1; k < n; k++)
			if (B[i][i] < B[k][i])
				for (j = 0; j <= n; j++)
				{
					double temp = B[i][j];
					B[i][j] = B[k][j];
					B[k][j] = temp;
				}

	for (i = 0; i < n - 1; i++)            //loop to perform the gauss elimination
		for (k = i + 1; k < n; k++)
		{
			double t = B[k][i] / B[i][i];
			for (j = 0; j <= n; j++)
				B[k][j] = B[k][j] - t * B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
		}
	for (i = n - 1; i >= 0; i--)                //back-substitution
	{                        //x is an array whose values correspond to the values of x,y,z..
		a[i] = B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
		for (j = 0; j < n; j++)
			if (j != i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
				a[i] = a[i] - B[i][j] * a[j];
		a[i] = a[i] / B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
	}
	//cout << "\nThe values of the coefficients are as follows:\n";
	//for (i = 0; i < n; i++)
	//	cout << "x^" << i << "=" << a[i] << endl;            // Print the values of x^0,x^1,x^2,x^3,....    
	//cout << "\nHence the fitted Polynomial is given by:\ny=";
	//for (i = 0; i < n; i++)
	//	cout << "(" << a[i] << ")" << "x^" << i << "+";
	//cout << "\n";

	return a;
}