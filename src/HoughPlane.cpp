#include "../inc/HoughPlane.hpp"
#include <fstream>
#include <iostream>
#include <tuple>
#include <math.h>
#define PI 3.14159265
using namespace std;

// Read the original point cloud data (in pts format), you need to allocate buffer and store x, y, z coordinates into struct data
bool readPointCloud(const char *filename, mydata &data)
{
	fstream inp(filename);
	if (!inp.good())
	{
		cerr << "\nError opening point cloud data file: " << filename;
		return false;
	}
	int nData;
	inp >> nData;

	// some pts file has a 'D' character after the number of points...:(
	char pp = inp.peek();
	if (pp == 'D' || pp == 'd')
		inp >> pp;

	// you may want to store the number of points somewhere in your struct
	// you should now allocate enough memory to store all the point-cloud data (x, y, z coordinates)...
	data.n = nData;
	data.pts = new float[3 * data.n];
	for (int i = 0; i < nData; ++i)
	{
		float x, y, z;
		string rest; // some pts files have only intensity, and some have RGB values, so we treat them all as a long string...
		inp >> x >> y >> z;
		std::getline(inp, rest);
		// now you have coordinates in x, y, z, and additional information in the string rest.  You need to store them into your data struct...
		data.pts[i * 3] = x;
		data.pts[i * 3 + 1] = y;
		data.pts[i * 3 + 2] = z;
	}
	inp.close();

	return true;
}

// This function translates all points in the point cloud so that all points that was in the AABB (axis-aligned bounding box) of [xmin, ymin, zmin] - [xmax, ymax, zmax]
// will be in the region of [-Lx, -Ly, -Lz] - [Lx, Ly, Lz], where Lx = 0.5*(xmax+xmin), Ly=0.5*(ymax+ymin), Lz=0.5*(zmax+zmin).
// In other words, the new AABB (after translation) will be centered at the origin (0, 0, 0).
// Also, this function returns sqrt( (0.5*(xmax-xmin))**2 + (0.5*(ymax-ymin))**2 + (0.5*(zmax-zmin))**2) ).  This is the maximum possible rho.
float centerPointCloudToOrigin(mydata &data)
{
	// cout << "\nYou need to implement something in " << __PRETTY_FUNCTION__ << " @ " << __LINE__ << " of " << __FILE__;
	// first, find bounds
	float tmpmax[3] = {data.pts[0], data.pts[1], data.pts[2]};
	float tmpmin[3] = {data.pts[0], data.pts[1], data.pts[2]};
	for (int whichn = 1; whichn < data.n; whichn++)
	{
		if (data.pts[whichn * 3] > tmpmax[0])
			tmpmax[0] = data.pts[whichn * 3];
		if (data.pts[whichn * 3 + 1] > tmpmax[1])
			tmpmax[1] = data.pts[whichn * 3 + 1];
		if (data.pts[whichn * 3 + 2] > tmpmax[2])
			tmpmax[2] = data.pts[whichn * 3 + 2];
		if (data.pts[whichn * 3] < tmpmin[0])
			tmpmin[0] = data.pts[whichn * 3];
		if (data.pts[whichn * 3 + 1] < tmpmin[1])
			tmpmin[1] = data.pts[whichn * 3 + 1];
		if (data.pts[whichn * 3 + 2] < tmpmin[2])
			tmpmin[2] = data.pts[whichn * 3 + 2];
	}
	// then, translate every point in the point-cloud (without create a new set of points)
	data.oldcen = new float[3];
	for (int i = 0; i < 3; i++)
	{
		data.oldcen[i] = (tmpmin[i] + tmpmax[i]) / 2;
	}
	// cout << data.oldcen[0] << data.oldcen[1] << data.oldcen[2] << endl;
	for (int whichn = 0; whichn < data.n; whichn++)
	{
		for (int whichd = 0; whichd < 3; whichd++)
		{
			data.pts[whichn * 3 + whichd] = data.pts[whichn * 3 + whichd] - data.oldcen[whichd];
		}
	}
	// also, record how much you have translated every point, because we need to un-translate them in the output
	// return maximum possible rho
	data.rhomax = sqrt(0.25 * (pow(tmpmax[0] - tmpmin[0], 2) +
							   pow(tmpmax[1] - tmpmin[1], 2) +
							   pow(tmpmax[2] - tmpmin[2], 2)));
	// cout << data.rhomax << endl;
	return data.rhomax;
}

// This function prepares and initializes the accumulator to store all votes with initial value of 0.
void prepareAccumulator(mydata &votes, const float rho_max, const size_t n_theta, const size_t n_phi, const size_t n_rho)
{
	votes.nrho = n_rho;
	votes.ntheta = n_theta;
	votes.nphi = n_phi;
	// votes.accumulator = new int **[n_rho + 1]();
	// for (int i = 0; i <= n_rho; i++)
	// {
	// 	votes.accumulator[i] = new int *[n_phi + 1];
	// 	for (int j = 0; j <= n_phi; j++)
	// 	{
	// 		votes.accumulator[i][j] = new int[n_theta];
	// 	}
	// }
	votes.accumulator = new int[n_theta * (n_phi + 1) * (n_rho + 1)]{0};
	// for (int i = 0; i <= n_rho; i++)
	// {
	// 	for (int j = 0; j <= n_phi; j++)
	// 	{
	// 		for (int k = 0; k < n_theta; k++)
	// 		{
	// 			votes.accumulator[i][j][k] = 0;
	// 		}
	// 	}
	// }
	// for (size_t i = 0; i < n_rho; i++)
	// {
	// 	for (size_t j = 0; j < n_theta; j++)
	// 	{
	// 		for (size_t k = 0; k < n_phi; k++)
	// 		{
	//			 cout << votes.accumulator[i][j][k] << endl;
	// 		}
	// 	}
	// }
	// cout << "\nYou need to implement something in " << __PRETTY_FUNCTION__ << " @ " << __LINE__ << " of " << __FILE__;
}

// This function conducts the Hough Transform to cast votes in the rho, theta, phi parametric space.
void houghTransform(mydata &data)
{
	// cout << "\nYou need to implement something in " << __PRETTY_FUNCTION__ << " @ " << __LINE__ << " of " << __FILE__;
	// float theta, phi, rho;
	float ctheta, cphi, stheta, sphi;
	float rho;
	int kappa;
	data.dtheta = PI / data.ntheta;
	// data.dtheta = (180.0f / data.ntheta) * (PI / 180.0f);
	data.dphi = (0.5 * PI) / data.nphi;
	// data.dphi = (90.0f / data.nphi) * (PI / 180.0f);
	data.drho = (2.0f * data.rhomax / data.nrho);
	float drho = data.nrho / (2.0f * data.rhomax);
	data.arrtheta = new float[data.ntheta * 2];
	data.arrphi = new float[(data.nphi + 1) * 2];
	// for (int whichtheta = 0; whichtheta < data.ntheta; whichtheta++)
	// {
	// 	// arrtheta[whichtheta] = whichtheta * data.dtheta;
	// 	data.arrtheta[whichtheta * 2] = cos(whichtheta * data.dtheta);
	// 	data.arrtheta[whichtheta * 2 + 1] = sin(whichtheta * data.dtheta);
	// }
	// for (int whichphi = 0; whichphi <= data.nphi; whichphi++)
	// {
	// 	// arrphi[whichphi] = whichphi * data.dphi;
	// 	data.arrphi[whichphi * 2] = sin(whichphi * data.dphi);
	// 	data.arrphi[whichphi * 2 + 1] = cos(whichphi * data.dphi);
	// }

	for (int whichphi = 0; whichphi <= data.nphi; whichphi++)
	{
		cphi = cos(whichphi * data.dphi);
		sphi = sin(whichphi * data.dphi);
		for (int whichtheta = 0; whichtheta < data.ntheta; whichtheta++)
		{
			ctheta = cos(whichtheta * data.dtheta);
			stheta = sin(whichtheta * data.dtheta);
			for (int whichpt = 0; whichpt < data.n * 3; whichpt += 3)
			{

				rho = data.pts[whichpt] * ctheta * sphi +
					  data.pts[whichpt + 1] * stheta * sphi +
					  data.pts[whichpt + 2] * cphi;
				// rho = data.pts[whichpt] * data.arrtheta[whichtheta * 2] * data.arrphi[whichphi * 2] +
				// 	  data.pts[whichpt + 1] * data.arrtheta[whichtheta * 2 + 1] * data.arrphi[whichphi * 2] +
				// 	  data.pts[whichpt + 2] * data.arrphi[whichphi * 2 + 1];
				kappa = (int)((rho + data.rhomax) * drho); //無條件捨去
				// kappa = (int)((rho + data.rhomax) / data.drho); //無條件捨去
				data.accumulator[whichtheta * (data.nrho + 1) * (data.nphi + 1) + whichphi * (data.nrho + 1) + kappa] += 1;
			}
		}
	}
	// cout << (2.0f * data.rhomax / data.nrho) << endl
	// 	 << 221 * (2.0f * data.rhomax / data.nrho) << endl
	// 	 << (180.0f / data.ntheta) * (PI / 180) << endl
	// 	 << (90.0f / data.nphi) * (PI / 180) << endl;
	// int all = 0;
	// for (size_t i = 0; i <= data.nrho; i++)
	// {
	// 	for (size_t j = 0; j <= data.nphi; j++)
	// 	{
	// 		for (size_t k = 0; k < data.ntheta; k++)
	// 		{
	// 			// cout << data.accumulator[i][j][k] << endl;
	// 			all += data.accumulator[i][j][k];
	// 		}
	// 	}
	// }
	// cout << all;
}

// find votes that are larger than threshold*total_number_of_points and store its Hough parameters (theta, phi, rho) into the data struct
void identifyPlaneParameters(mydata &data, const float threshold)
{
	// cout << "\nYou need to implement something in " << __PRETTY_FUNCTION__ << " @ " << __LINE__ << " of " << __FILE__;
	vector<int> sorts;
	for (int whichtheta = 0; whichtheta < data.ntheta; whichtheta++)
	{
		for (int whichphi = 0; whichphi <= data.nphi; whichphi++)
		{
			for (int whichrho = 0; whichrho <= data.nrho; whichrho++)
			{
				if (data.accumulator[whichtheta * (data.nrho + 1) * (data.nphi + 1) + whichphi * (data.nrho + 1) + whichrho] > threshold * data.n)
				{
					// cout << data.accumulator[whichtheta * (data.nrho + 1) * (data.nphi + 1) + whichphi * (data.nrho + 1) + whichrho] << "," << whichrho << "," << whichphi << "," << whichtheta << endl;
					// cout << data.accumulator[whichrho][whichphi][whichtheta] << endl;
					// data.output.push_back(whichrho);
					// data.output.push_back(whichphi);
					// data.output.push_back(whichtheta);
					// int vote = data.accumulator[whichrho][whichphi][whichtheta];
					int vote = data.accumulator[whichtheta * (data.nrho + 1) * (data.nphi + 1) + whichphi * (data.nrho + 1) + whichrho];
					bool add = false;
					int s = sorts.size();
					float theta, phi;
					phi = whichphi * data.dphi;
					theta = whichtheta * data.dtheta;
					for (int j = 0; j < s; j++)
					{
						if (vote > sorts[j])
						{
							sorts.insert(sorts.begin() + j, vote);
							data.output.insert(data.output.begin() + (j * 4), {cos(theta) * sin(phi), sin(theta) * sin(phi), cos(phi), whichrho * data.drho - data.rhomax});
							// data.output.insert(data.output.begin() + j * 3, {whichrho, whichphi, whichtheta});
							// data.output.insert(data.output.begin() + (j * 3 + 1), whichphi);
							// data.output.insert(data.output.begin() + (j * 3 + 2), whichtheta);
							add = true;
							break;
						}
					}
					if (add == false)
					{
						sorts.push_back(vote);
						data.output.push_back(cos(theta) * sin(phi));
						data.output.push_back(sin(theta) * sin(phi));
						data.output.push_back(cos(phi));
						data.output.push_back(whichrho * data.drho - data.rhomax);
						// data.output.push_back(whichrho);
						// data.output.push_back(whichphi);
						// data.output.push_back(whichtheta);
					}
				}
			}
		}
	}
	// for (size_t i = 0; i < data.output.size() / 4; i++)
	// {
	// 	cout << data.output[i * 4] << ", ";
	// 	cout << data.output[i * 3 + 1] << ", ";
	// 	cout << data.output[i * 3 + 2] << ",";
	// 	cout << data.output[i * 3 + 3] << endl;
	// }
}

// Task 8a - Output point cloud data in ptx format
bool outputPtxFile(const mydata &data, const char *outputCloudData)
{
	// cout << "\nYou need to implement something in " << __PRETTY_FUNCTION__ << " @ " << __LINE__ << " of " << __FILE__;
	ofstream outp(outputCloudData);
	if (!outp)
		return false;
	// Output header of PLY file format, which includes the number of points
	outp << "ply\nformat ascii 1.0\nelement vertex " << data.n
		 << "\nproperty float x\nproperty float y\nproperty float z"
		 << "\nproperty uchar red\nproperty uchar green\nproperty uchar blue"
		 << "\nend_header";

	// https://medium.com/@elope139/reading-ply-files-that-are-in-binary-format-cab3a37276a2
	// http://gamma.cs.unc.edu/POWERPLANT/papers/ply.pdf
	// https://codeyarns.com/2011/08/20/convert-between-ascii-and-binary-ply-file-formats/

	// go through every point's coordinate and RGB color
	vector<int> R(data.n, 0);
	vector<int> G(data.n, 0);
	vector<int> B(data.n, 0);

	for (size_t whichplane = 0; whichplane < data.output.size(); whichplane += 4)
	{
		// int rgb[] = {0, 0, 0};											 // R: [0, 255], G: [0, 255], B: [0, 255], RGB values are determined by their Hough parameters (a.k.a. normal direction)
		// uint8_t rgb[] = {0, 0, 0};									// R: [0, 255], G: [0, 255], B: [0, 255], RGB values are determined by their Hough parameters (a.k.a. normal direction)
		// You need to put right data into xyz and rgb arrays...

		for (int whichpt = 0; whichpt < data.n; ++whichpt)
		{
			// float xyz[] = {
			// 	data.pts[whichpt * 3],
			// 	data.pts[whichpt * 3 + 1],
			// 	data.pts[whichpt * 3 + 2],
			// }; // x, y, z, do remember to translate them back to their original position
			// float theta, phi;
			// phi = data.output[whichplane * 3 + 1] * data.dphi;
			// theta = data.output[whichplane * 3 + 2] * data.dtheta;
			if (abs(data.pts[whichpt * 3] * data.output[whichplane] +
					data.pts[whichpt * 3 + 1] * data.output[whichplane + 1] +
					data.pts[whichpt * 3 + 2] * data.output[whichplane + 2] - data.output[whichplane + 3]) < data.drho &&
				R[whichpt] == 0 && G[whichpt] == 0 && B[whichpt] == 0)
			{
				R[whichpt] = abs(255 * data.output[whichplane]);
				G[whichpt] = abs(255 * data.output[whichplane + 1]);
				B[whichpt] = abs(255 * data.output[whichplane + 2]);
				outp << "\n"
					 << data.pts[whichpt * 3] + data.oldcen[0] << " " << data.pts[whichpt * 3 + 1] + data.oldcen[1] << " " << data.pts[whichpt * 3 + 2] + data.oldcen[2] << " " << R[whichpt] << " " << G[whichpt] << " " << B[whichpt];
			}
		}
	}
	for (auto whichpt = 0; whichpt < data.n; ++whichpt)
	{
		// float xyz[] = {data.pts[whichpt * 3], data.pts[whichpt * 3 + 1], data.pts[whichpt * 3 + 2]};
		if (R[whichpt] == 0 && G[whichpt] == 0 && B[whichpt] == 0)
		{
			outp << "\n"
				 << data.pts[whichpt * 3] + data.oldcen[0] << " " << data.pts[whichpt * 3 + 1] + data.oldcen[1] << " " << data.pts[whichpt * 3 + 2] + data.oldcen[2] << " " << 0 << " " << 0 << " " << 0;
		};
	}

	outp.close();

	return true;
}

// task 10 - release all allocated memory
void release(mydata &data)
{
	// cout << "\nYou need to implement something in " << __PRETTY_FUNCTION__ << " @ " << __LINE__ << " of " << __FILE__;
	delete[] data.pts;
	delete[] data.oldcen;
	delete[] data.accumulator;
	delete[] data.arrtheta;
	delete[] data.arrphi;
}
