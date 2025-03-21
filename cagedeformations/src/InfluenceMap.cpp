#include <cagedeformations/InfluenceMap.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>

/// Modified from libigl
bool writeOBJVertexColors(
	const std::string& str,
	const Eigen::MatrixXd& V,
	const Eigen::MatrixXd& V_colors,
	const Eigen::MatrixXi& F)
{
	using namespace std;
	assert(V.cols() == 3 && "V should have 3 columns");
	ofstream s(str);
	if (!s.is_open())
	{
		fprintf(stderr, "IOError: writeOBJ() could not open %s\n", str.c_str());
		return false;
	}

	for (int i = 0; i < V.rows(); ++i)
	{
		const Eigen::Vector3d vert = V.row(i);
		const Eigen::Vector3d col = V_colors.row(i);

		s << "v " << std::fixed << std::setprecision(17) << vert.x() << " " <<
			std::fixed << std::setprecision(17) << vert.y() << " " <<
			std::fixed << std::setprecision(17) << vert.z() << " " <<
			std::fixed << std::setprecision(17) << col.x() << " " <<
			std::fixed << std::setprecision(17) << col.y() << " " <<
			std::fixed << std::setprecision(17) << col.z() << " \n";
	}

	s << (F.array() + 1).format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "f ", "", "", "\n"));

	return true;
}


Eigen::Vector3d HSVtoRGB(double H, double S, double V) {
	if (H > 360 || H < 0 || S>100 || S < 0 || V>100 || V < 0) {
		std::cout << "The given HSV values are not in valid range (H,S,V) = (" << H << ", " << S << ", " << V << ")\n";
		assert(false);
		return Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN());
	}
	double s = S / 100.;
	double v = V / 100.;
	double C = s * v;
	double X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
	double m = v - C;
	double r = 0, g = 0, b = 0;
	if (H >= 0. && H < 60.) {
		r = C, g = X, b = 0.;
	}
	else if (H >= 60. && H < 120.) {
		r = X, g = C, b = 0.;
	}
	else if (H >= 120. && H < 180.) {
		r = 0, g = C, b = X;
	}
	else if (H >= 180. && H < 240.) {
		r = 0, g = X, b = C;
	}
	else if (H >= 240. && H < 300.) {
		r = X, g = 0, b = C;
	}
	else {
		r = C, g = 0, b = X;
	}
	double R = r + m;
	double G = g + m;
	double B = b + m;

	return Eigen::Vector3d(R, G, B);
}

void write_influence_color_map_OBJ(const std::string & file_name, const Eigen::MatrixXd & V,
	const Eigen::MatrixXi & T, const Eigen::MatrixXd & W, const std::vector<int> & control_vertices_idx, int cage_vertices_offset, bool transposeW)
{
	Eigen::MatrixXd V_colors;
	Eigen::VectorXd influences(V.rows());
	V_colors.resize(V.rows(), 3);
	for (int i = 0; i < V.rows(); ++i)
	{
		auto embedding_idx = i + cage_vertices_offset;
		double res = 0.;
		for (auto idx : control_vertices_idx)
		{
			res += (transposeW) ? W(idx, embedding_idx) : W(embedding_idx, idx);
		}
		influences(i) = res;
	}

	for (int i = 0; i < V.rows(); ++i)
	{
		auto interpolate = [&](double val)
		{
			val = std::min(std::max(0., val), 1.) * 100;
			val = std::log(1 + val) / std::log(1 + 100);
			assert(val >= 0. && val <= 1.);
			return val;
			//return (val - min_influence) / (max_influence - min_influence);
		};

		V_colors.row(i) = HSVtoRGB(240. * (1. - interpolate(influences(i))), 100., 100.);
	}

	writeOBJVertexColors(file_name, V, V_colors, T);
}