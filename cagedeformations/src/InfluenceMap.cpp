#include <cagedeformations/InfluenceMap.h>

#include <iostream>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

/// Modified from libigl
bool writeOBJVertexColors(
	const std::string& str,
	const Eigen::MatrixXd& V,
	const Eigen::MatrixXd& V_colors,
	const Eigen::MatrixXi& F,
	const std::vector<std::string>& header_comments)
{
	using namespace std;
	assert(V.cols() == 3 && "V should have 3 columns");
	ofstream s(str);
	if (!s.is_open())
	{
		fprintf(stderr, "IOError: writeOBJ() could not open %s\n", str.c_str());
		return false;
	}

	for (const auto& comment : header_comments)
	{
		s << "# " << comment << "\n";
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

Eigen::Vector3d viridisToRGB(double t)
{
	// Polynomial fit of the matplotlib viridis colormap, https://www.shadertoy.com/view/WlfXRN.
	static const Eigen::Vector3d c0(0.2777273272234177, 0.005407344544966578, 0.3340998053353061);
	static const Eigen::Vector3d c1(0.1050930431085774, 1.404613529898575, 1.384590162594685);
	static const Eigen::Vector3d c2(-0.3308618287255563, 0.214847559468213, 0.09509516302823659);
	static const Eigen::Vector3d c3(-4.634230498983486, -5.799100973351585, -19.33244095627987);
	static const Eigen::Vector3d c4(6.228269936347081, 14.17993336680509, 56.69055260068105);
	static const Eigen::Vector3d c5(4.776384997670288, -13.74514537774601, -65.35303263337234);
	static const Eigen::Vector3d c6(-5.435455855934631, 4.645852612178535, 26.3124352495832);

	t = std::isfinite(t) ? std::min(std::max(0., t), 1.) : 0.;

	const Eigen::Vector3d color = c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * (c5 + t * c6)))));

	return color.cwiseMax(0.).cwiseMin(1.);
}

namespace
{
	/// Stroke of the isolines. Viridis runs from a dark blue-purple to a bright yellow and
	/// contains no magenta at all, so a saturated magenta keeps its hue contrast over the
	/// whole gradient, and its luminance sits between the two ends: clearly brighter than
	/// the dark end and clearly darker than the yellow end.
	const Eigen::Vector3d kContourStroke(0.90, 0.10, 0.45);

	/// Emphasized isolines keep the hue and go deeper, on top of being drawn twice as thick.
	const Eigen::Vector3d kEmphasizedContourStroke(0.70, 0.05, 0.35);

	/// Half width of an isoline in local vertex spacings. A distance field has a unit
	/// gradient, so this is also the width of the band in world units. Three quarters of a
	/// spacing keeps the lines closed no matter how the field runs across the triangles
	/// (along a diagonal the vertices are the furthest apart), the emphasized ones are
	/// twice as wide on top of their deeper stroke.
	constexpr double kContourHalfWidthInSpacings = 0.75;
	constexpr double kEmphasizedContourHalfWidthInSpacings = 1.5;

	/// Cap of the half width relative to the isoline spacing, so that neighboring isolines
	/// can never merge into a solid color on a mesh that is coarse for the chosen interval.
	constexpr double kMaxContourHalfWidthInIntervals = 0.2;
	constexpr double kMaxEmphasizedContourHalfWidthInIntervals = 0.35;

	/// Number of isolines the automatic spacing aims for over the normalization range.
	constexpr int kAutoContourCount = 20;

	/// Local vertex spacing (relative to the interval) up to which the mesh can still
	/// resolve the isolines. Below that resolution the vertices alias against the isolines
	/// instead of sampling them, so no line is drawn at all rather than a smeared one.
	constexpr double kMaxResolvableSpacingInIntervals = 0.5;

	/// Mean length of the edges incident to each vertex, the local resolution of the mesh.
	Eigen::VectorXd computeVertexSpacings(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
	{
		Eigen::VectorXd totalLengths = Eigen::VectorXd::Zero(V.rows());
		Eigen::VectorXd edgeCounts = Eigen::VectorXd::Zero(V.rows());

		for (int f = 0; f < F.rows(); ++f)
		{
			for (int c = 0; c < F.cols(); ++c)
			{
				const int from = F(f, c);
				const int to = F(f, (c + 1) % F.cols());
				if (from < 0 || to < 0 || from >= V.rows() || to >= V.rows())
				{
					continue;
				}

				const double length = (V.row(from).leftCols<3>() - V.row(to).leftCols<3>()).norm();
				totalLengths(from) += length;
				edgeCounts(from) += 1.;
				totalLengths(to) += length;
				edgeCounts(to) += 1.;
			}
		}

		// Vertices without an incident face fall back to the mean spacing of the mesh.
		const double meanSpacing = (edgeCounts.sum() > 0.) ? (totalLengths.sum() / edgeCounts.sum()) : 0.;

		Eigen::VectorXd spacings(V.rows());
		for (int i = 0; i < V.rows(); ++i)
		{
			spacings(i) = (edgeCounts(i) > 0.) ? (totalLengths(i) / edgeCounts(i)) : meanSpacing;
		}

		return spacings;
	}

	std::string formatValue(const double value)
	{
		std::ostringstream stream;
		stream << std::setprecision(6) << value;

		return stream.str();
	}

	std::string formatOrdinal(const int value)
	{
		const int lastDigit = value % 10;
		const int lastTwoDigits = value % 100;
		const char* suffix = "th";
		if (lastTwoDigits < 11 || lastTwoDigits > 13)
		{
			suffix = (lastDigit == 1) ? "st" : (lastDigit == 2) ? "nd" : (lastDigit == 3) ? "rd" : "th";
		}

		return std::to_string(value) + suffix;
	}
}

void write_distance_color_map_OBJ(const std::string& file_name, const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& T, const Eigen::VectorXd& distances, const DistanceColorMapParams& params)
{
	assert(distances.size() == V.rows() && "One distance per vertex is required");

	double dataMax = 0.;
	for (int i = 0; i < V.rows(); ++i)
	{
		if (std::isfinite(distances(i)))
		{
			dataMax = std::max(dataMax, distances(i));
		}
	}

	// Normalizing against the maximum of the data set makes a single export easy to read,
	// a fixed maximum keeps separate exports comparable.
	const bool hasFixedMax = (params.maxDistance > 0.);
	const double normalizationMax = hasFixedMax ? params.maxDistance : dataMax;
	const double safeNormalizationMax = (normalizationMax > 0.) ? normalizationMax : 1.;
	const double interval = (params.contourInterval < 0.)
		? (safeNormalizationMax / kAutoContourCount)
		: params.contourInterval;
	const int emphasisEvery = params.contourEmphasisEvery;

	Eigen::MatrixXd V_colors(V.rows(), 3);
	for (int i = 0; i < V.rows(); ++i)
	{
		const double distance = std::isfinite(distances(i)) ? std::max(0., distances(i)) : 0.;
		V_colors.row(i) = viridisToRGB(distance / safeNormalizationMax);
	}

	if (interval > 0.)
	{
		const Eigen::VectorXd spacings = computeVertexSpacings(V, T);

		for (int i = 0; i < V.rows(); ++i)
		{
			if (spacings(i) > kMaxResolvableSpacingInIntervals * interval)
			{
				continue;
			}

			const double distance = std::isfinite(distances(i)) ? std::max(0., distances(i)) : 0.;
			const long long isolineIndex = std::llround(distance / interval);
			const double offset = std::abs(distance - static_cast<double>(isolineIndex) * interval);
			const bool isEmphasized = (emphasisEvery >= 2) && (isolineIndex % emphasisEvery == 0);
			const double halfWidth = isEmphasized
				? std::min(kEmphasizedContourHalfWidthInSpacings * spacings(i), kMaxEmphasizedContourHalfWidthInIntervals * interval)
				: std::min(kContourHalfWidthInSpacings * spacings(i), kMaxContourHalfWidthInIntervals * interval);

			if (offset <= halfWidth)
			{
				V_colors.row(i) = isEmphasized ? kEmphasizedContourStroke : kContourStroke;
			}
		}
	}

	std::vector<std::string> headerComments;
	headerComments.push_back("Distance field color map" + (params.label.empty() ? std::string() : ": " + params.label));
	headerComments.push_back("Gradient: viridis over [0, " + formatValue(safeNormalizationMax) + "] ("
		+ (hasFixedMax ? "fixed maximum" : "maximum of the data set") + ")");
	headerComments.push_back("Largest distance in the data set: " + formatValue(dataMax));
	headerComments.push_back(interval > 0.
		? "Isolines: every " + formatValue(interval) + " world units"
			+ (emphasisEvery >= 2 ? ", every " + formatOrdinal(emphasisEvery) + " emphasized" : ", none emphasized")
		: std::string("Isolines: none"));

	writeOBJVertexColors(file_name, V, V_colors, T, headerComments);
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
		};

		V_colors.row(i) = HSVtoRGB(240. * (1. - interpolate(influences(i))), 100., 100.);
	}

	writeOBJVertexColors(file_name, V, V_colors, T);
}
