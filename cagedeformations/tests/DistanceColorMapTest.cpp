/**
 * Validation harness for the distance field color map (the interior / Euclidean distance
 * visualization that follows the influence map export).
 *
 * Gradient acceptance: the viridis ramp has to match the tabulated colormap at its ends
 *   and stay perceptually uniform, meaning a monotonically rising luminance.
 * Normalization acceptance: without a fixed maximum the largest distance of the data set
 *   has to land on the top of the ramp, and with a fixed maximum equal distances have to
 *   receive equal colors across exports of different data sets.
 * Contour acceptance: the isolines have to sit on the multiples of the interval, every
 *   n-th of them has to be emphasized and drawn wider, and the stroke has to keep its
 *   contrast against both ends of the gradient.
 * Output acceptance: the written file has to stay a vertex colored OBJ with the same
 *   vertex and face count as the input mesh, labeled by a header comment.
 */

#include <cagedeformations/InfluenceMap.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace
{

int failures = 0;

void check(bool condition, const std::string& what)
{
	if (condition)
	{
		std::cout << "[PASS] " << what << std::endl;
	}
	else
	{
		std::cout << "[FAIL] " << what << std::endl;
		++failures;
	}
}

// --------------------------------------------------------------------------------------
// Test mesh and helpers.
// --------------------------------------------------------------------------------------

struct TestMesh
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
};

/// Regular triangulated grid in the xy plane with `columns` x `rows` vertices, spaced by
/// `spacing`. The x coordinate of a vertex is used as a distance field with unit gradient,
/// which is what an interior or Euclidean distance field looks like locally.
TestMesh makeGrid(const int columns, const int rows, const double spacing)
{
	TestMesh mesh;
	mesh.V.resize(columns * rows, 3);
	for (int y = 0; y < rows; ++y)
	{
		for (int x = 0; x < columns; ++x)
		{
			mesh.V.row(y * columns + x) << static_cast<double>(x) * spacing, static_cast<double>(y) * spacing, 0.;
		}
	}

	mesh.F.resize(2 * (columns - 1) * (rows - 1), 3);
	int face = 0;
	for (int y = 0; y + 1 < rows; ++y)
	{
		for (int x = 0; x + 1 < columns; ++x)
		{
			const int v00 = y * columns + x;
			const int v10 = v00 + 1;
			const int v01 = v00 + columns;
			const int v11 = v01 + 1;
			mesh.F.row(face++) << v00, v10, v11;
			mesh.F.row(face++) << v00, v11, v01;
		}
	}

	return mesh;
}

struct ParsedOBJ
{
	std::vector<std::string> comments;
	std::vector<Eigen::Vector3d> positions;
	std::vector<Eigen::Vector3d> colors;
	int faceCount = 0;
};

ParsedOBJ parseOBJ(const std::string& fileName)
{
	ParsedOBJ parsed;
	std::ifstream stream(fileName);
	std::string line;
	while (std::getline(stream, line))
	{
		if (line.rfind("# ", 0) == 0)
		{
			parsed.comments.push_back(line.substr(2));
			continue;
		}

		std::istringstream lineStream(line);
		std::string keyword;
		lineStream >> keyword;
		if (keyword == "v")
		{
			Eigen::Vector3d position = Eigen::Vector3d::Zero();
			Eigen::Vector3d color = Eigen::Vector3d::Zero();
			lineStream >> position.x() >> position.y() >> position.z() >> color.x() >> color.y() >> color.z();
			parsed.positions.push_back(position);
			parsed.colors.push_back(color);
		}
		else if (keyword == "f")
		{
			++parsed.faceCount;
		}
	}

	return parsed;
}

/// Relative luminance of an sRGB color, as defined by WCAG.
double relativeLuminance(const Eigen::Vector3d& color)
{
	const auto linearize = [](const double channel)
	{
		return (channel <= 0.04045) ? (channel / 12.92) : std::pow((channel + 0.055) / 1.055, 2.4);
	};

	return 0.2126 * linearize(color.x()) + 0.7152 * linearize(color.y()) + 0.0722 * linearize(color.z());
}

double contrastRatio(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs)
{
	const double a = relativeLuminance(lhs);
	const double b = relativeLuminance(rhs);

	return (std::max(a, b) + 0.05) / (std::min(a, b) + 0.05);
}

/// The color the writer gives to a vertex, read back from a temporary export.
std::vector<Eigen::Vector3d> colorsOf(const TestMesh& mesh, const Eigen::VectorXd& distances,
	const DistanceColorMapParams& params, const std::string& fileName)
{
	write_distance_color_map_OBJ(fileName, mesh.V, mesh.F, distances, params);

	return parseOBJ(fileName).colors;
}

// --------------------------------------------------------------------------------------
// Tests.
// --------------------------------------------------------------------------------------

void testGradient()
{
	std::cout << "\n=== Gradient: perceptually uniform viridis ramp ===" << std::endl;

	// Ends of the tabulated matplotlib viridis colormap.
	const Eigen::Vector3d tabulatedLow(0.267004, 0.004874, 0.329415);
	const Eigen::Vector3d tabulatedHigh(0.993248, 0.906157, 0.143936);

	check((viridisToRGB(0.) - tabulatedLow).cwiseAbs().maxCoeff() < 0.02, "viridis starts at the tabulated dark blue");
	check((viridisToRGB(1.) - tabulatedHigh).cwiseAbs().maxCoeff() < 0.02, "viridis ends at the tabulated yellow");

	// Out of range values must not produce colors outside the ramp.
	check(viridisToRGB(-1.) == viridisToRGB(0.), "viridis clamps below the range");
	check(viridisToRGB(2.) == viridisToRGB(1.), "viridis clamps above the range");
	check(viridisToRGB(std::numeric_limits<double>::quiet_NaN()) == viridisToRGB(0.), "viridis maps NaN to the bottom of the ramp");

	bool inUnitCube = true;
	bool monotoneLuminance = true;
	double previousLuminance = -1.;
	double minStep = std::numeric_limits<double>::max();
	double maxStep = 0.;
	for (int i = 0; i <= 256; ++i)
	{
		const double t = static_cast<double>(i) / 256.;
		const Eigen::Vector3d color = viridisToRGB(t);
		inUnitCube = inUnitCube && color.minCoeff() >= 0. && color.maxCoeff() <= 1.;

		const double luminance = relativeLuminance(color);
		if (previousLuminance >= 0.)
		{
			const double step = luminance - previousLuminance;
			monotoneLuminance = monotoneLuminance && step > 0.;
			minStep = std::min(minStep, step);
			maxStep = std::max(maxStep, step);
		}
		previousLuminance = luminance;
	}

	check(inUnitCube, "viridis stays inside the unit RGB cube");
	check(monotoneLuminance, "viridis luminance rises monotonically (perceptually uniform, no rainbow banding)");
	std::cout << "luminance step over the ramp: min=" << minStep << " max=" << maxStep << std::endl;
}

void testNormalization()
{
	std::cout << "\n=== Normalization: data set maximum and fixed maximum ===" << std::endl;

	const TestMesh mesh = makeGrid(21, 3, 0.05);
	Eigen::VectorXd distances(mesh.V.rows());
	for (int i = 0; i < mesh.V.rows(); ++i)
	{
		distances(i) = mesh.V(i, 0);
	}
	const double dataMax = distances.maxCoeff();

	DistanceColorMapParams params;
	params.contourInterval = 0.;

	const auto autoColors = colorsOf(mesh, distances, params, "distance_map_auto.obj");
	int farthest = 0;
	for (int i = 0; i < mesh.V.rows(); ++i)
	{
		farthest = (distances(i) > distances(farthest)) ? i : farthest;
	}

	check((autoColors[farthest] - viridisToRGB(1.)).cwiseAbs().maxCoeff() < 1e-6,
		"without a fixed maximum the largest distance lands on the top of the ramp");
	check((autoColors[0] - viridisToRGB(0.)).cwiseAbs().maxCoeff() < 1e-6,
		"the source point lands on the bottom of the ramp");

	// The same field normalized against twice its maximum has to stop halfway up the ramp.
	DistanceColorMapParams fixedParams = params;
	fixedParams.maxDistance = 2. * dataMax;
	const auto fixedColors = colorsOf(mesh, distances, fixedParams, "distance_map_fixed.obj");
	check((fixedColors[farthest] - viridisToRGB(0.5)).cwiseAbs().maxCoeff() < 1e-6,
		"a fixed maximum scales the ramp instead of the data set");

	// Half of the mesh exported on its own: with a fixed maximum every distance keeps the
	// color it had in the full export, which is what makes separate exports comparable.
	Eigen::VectorXd halfDistances = distances;
	for (int i = 0; i < halfDistances.size(); ++i)
	{
		halfDistances(i) = std::min(halfDistances(i), 0.5 * dataMax);
	}
	const auto halfFixedColors = colorsOf(mesh, halfDistances, fixedParams, "distance_map_fixed_half.obj");
	const auto halfAutoColors = colorsOf(mesh, halfDistances, params, "distance_map_auto_half.obj");

	bool fixedIsComparable = true;
	bool autoIsRescaled = false;
	for (int i = 0; i < halfDistances.size(); ++i)
	{
		if (halfDistances(i) == distances(i))
		{
			fixedIsComparable = fixedIsComparable && (halfFixedColors[i] - fixedColors[i]).cwiseAbs().maxCoeff() < 1e-6;
			autoIsRescaled = autoIsRescaled || (halfAutoColors[i] - autoColors[i]).cwiseAbs().maxCoeff() > 1e-3;
		}
	}

	check(fixedIsComparable, "a fixed maximum gives equal distances equal colors across exports");
	check(autoIsRescaled, "the data set maximum rescales per export (why the fixed maximum exists)");
}

void testContours()
{
	std::cout << "\n=== Contours: isolines every interval world units ===" << std::endl;

	const double spacing = 0.02;
	const double interval = 0.1;
	const int emphasisEvery = 5;
	const TestMesh mesh = makeGrid(51, 3, spacing);
	Eigen::VectorXd distances(mesh.V.rows());
	for (int i = 0; i < mesh.V.rows(); ++i)
	{
		distances(i) = mesh.V(i, 0);
	}

	DistanceColorMapParams params;
	params.contourInterval = interval;
	params.contourEmphasisEvery = emphasisEvery;
	params.label = "test field";
	const auto colors = colorsOf(mesh, distances, params, "distance_map_contours.obj");

	// A stroked vertex is one whose color left the gradient.
	const auto isOnGradient = [&](const int vertex)
	{
		const Eigen::Vector3d expected = viridisToRGB(distances(vertex) / distances.maxCoeff());

		return (colors[vertex] - expected).cwiseAbs().maxCoeff() < 1e-6;
	};

	std::vector<int> strokeWidthPerIsoline(11, 0);
	std::vector<int> distinctStrokeColors;
	bool strokesSitOnIsolines = true;
	int strokedVertices = 0;
	for (int i = 0; i < mesh.V.rows(); ++i)
	{
		if (isOnGradient(i))
		{
			continue;
		}

		++strokedVertices;
		const double distance = distances(i);
		const int isoline = static_cast<int>(std::llround(distance / interval));
		strokesSitOnIsolines = strokesSitOnIsolines && std::abs(distance - isoline * interval) <= 2. * spacing;
		if (mesh.V(i, 1) == 0.)
		{
			++strokeWidthPerIsoline[isoline];
		}
	}

	check(strokedVertices > 0, "isolines are drawn into the vertex colors");
	check(strokesSitOnIsolines, "every stroked vertex sits on a multiple of the interval");

	bool allIsolinesDrawn = true;
	bool emphasizedAreThicker = true;
	for (std::size_t isoline = 0; isoline < strokeWidthPerIsoline.size(); ++isoline)
	{
		allIsolinesDrawn = allIsolinesDrawn && strokeWidthPerIsoline[isoline] > 0;
		if (isoline % emphasisEvery == 0)
		{
			continue;
		}

		emphasizedAreThicker = emphasizedAreThicker && strokeWidthPerIsoline[isoline] < strokeWidthPerIsoline[0];
	}

	std::cout << "isoline width in vertices:";
	for (const auto width : strokeWidthPerIsoline)
	{
		std::cout << " " << width;
	}
	std::cout << std::endl;

	check(allIsolinesDrawn, "all 11 isolines of the field are drawn (bands stay countable)");
	check(emphasizedAreThicker, "every 5th isoline is drawn thicker than the ones in between");

	// The emphasized isolines also differ in color from the ones in between.
	const auto strokeAt = [&](const double distance)
	{
		int closest = 0;
		for (int i = 0; i < mesh.V.rows(); ++i)
		{
			if (std::abs(distances(i) - distance) < std::abs(distances(closest) - distance))
			{
				closest = i;
			}
		}

		return colors[closest];
	};

	const Eigen::Vector3d stroke = strokeAt(0.1);
	const Eigen::Vector3d emphasizedStroke = strokeAt(0.5);
	check((stroke - emphasizedStroke).cwiseAbs().maxCoeff() > 0.05, "emphasized isolines use a deeper stroke");
	check(relativeLuminance(emphasizedStroke) < relativeLuminance(stroke), "the emphasized stroke is the darker one");

	// Legibility of the stroke against the gradient it is drawn on.
	double minContrastAtEnds = std::numeric_limits<double>::max();
	double minDistanceToRamp = std::numeric_limits<double>::max();
	for (const auto& strokeColor : { stroke, emphasizedStroke })
	{
		minContrastAtEnds = std::min({ minContrastAtEnds, contrastRatio(strokeColor, viridisToRGB(0.)), contrastRatio(strokeColor, viridisToRGB(1.)) });
		for (int i = 0; i <= 256; ++i)
		{
			minDistanceToRamp = std::min(minDistanceToRamp, (strokeColor - viridisToRGB(static_cast<double>(i) / 256.)).norm());
		}
	}

	std::cout << "stroke contrast at the ends of the gradient: " << minContrastAtEnds
		<< ", minimum RGB distance to the gradient: " << minDistanceToRamp << std::endl;
	check(minContrastAtEnds > 2., "both strokes keep their contrast at the dark and the bright end of the gradient");
	check(minDistanceToRamp > 0.4, "no stroke color is close to any color of the gradient");

	// An interval below the mesh resolution must not flood the map with stroke: the mesh
	// cannot resolve those isolines, so none of them is drawn.
	DistanceColorMapParams denseParams = params;
	denseParams.contourInterval = 0.1 * spacing;
	const auto denseColors = colorsOf(mesh, distances, denseParams, "distance_map_contours_dense.obj");
	bool denseStaysOnGradient = true;
	for (int i = 0; i < mesh.V.rows(); ++i)
	{
		const Eigen::Vector3d expected = viridisToRGB(distances(i) / distances.maxCoeff());
		denseStaysOnGradient = denseStaysOnGradient && (denseColors[i] - expected).cwiseAbs().maxCoeff() < 1e-6;
	}
	check(denseStaysOnGradient, "an interval below the mesh resolution does not flood the map with stroke");

	// Isolines can be turned off completely.
	DistanceColorMapParams noContourParams = params;
	noContourParams.contourInterval = 0.;
	const auto plainColors = colorsOf(mesh, distances, noContourParams, "distance_map_no_contours.obj");
	bool allOnGradient = true;
	for (int i = 0; i < mesh.V.rows(); ++i)
	{
		const Eigen::Vector3d expected = viridisToRGB(distances(i) / distances.maxCoeff());
		allOnGradient = allOnGradient && (plainColors[i] - expected).cwiseAbs().maxCoeff() < 1e-6;
	}
	check(allOnGradient, "a zero interval draws no isolines at all");
}

void testOutputFormat()
{
	std::cout << "\n=== Output: vertex colored OBJ, labeled by its header ===" << std::endl;

	const TestMesh mesh = makeGrid(11, 11, 0.1);
	Eigen::VectorXd distances(mesh.V.rows());
	for (int i = 0; i < mesh.V.rows(); ++i)
	{
		distances(i) = mesh.V.row(i).norm();
	}

	DistanceColorMapParams interiorParams;
	interiorParams.label = "interior distance to cage vertex 7";
	write_distance_color_map_OBJ("distance_map_interior.obj", mesh.V, mesh.F, distances, interiorParams);

	DistanceColorMapParams euclideanParams;
	euclideanParams.label = "euclidean distance to cage vertex 7";
	euclideanParams.maxDistance = 2.;
	write_distance_color_map_OBJ("distance_map_euclidean.obj", mesh.V, mesh.F, distances, euclideanParams);

	const ParsedOBJ interior = parseOBJ("distance_map_interior.obj");
	const ParsedOBJ euclidean = parseOBJ("distance_map_euclidean.obj");

	check(static_cast<int>(interior.positions.size()) == mesh.V.rows(), "the export keeps every mesh vertex");
	check(interior.faceCount == static_cast<int>(mesh.F.rows()), "the export keeps every mesh face");

	bool positionsMatch = true;
	bool colorsInRange = true;
	for (std::size_t i = 0; i < interior.positions.size(); ++i)
	{
		positionsMatch = positionsMatch && (interior.positions[i] - mesh.V.row(i).transpose()).cwiseAbs().maxCoeff() < 1e-9;
		colorsInRange = colorsInRange && interior.colors[i].minCoeff() >= 0. && interior.colors[i].maxCoeff() <= 1.;
	}

	check(positionsMatch, "the exported vertex positions are unchanged");
	check(colorsInRange, "the exported vertex colors stay inside the unit RGB cube");

	const auto hasComment = [](const ParsedOBJ& parsed, const std::string& needle)
	{
		for (const auto& comment : parsed.comments)
		{
			if (comment.find(needle) != std::string::npos)
			{
				return true;
			}
		}

		return false;
	};

	check(hasComment(interior, "interior distance to cage vertex 7"), "the interior export is labeled in its header");
	check(hasComment(euclidean, "euclidean distance to cage vertex 7"), "the euclidean export is labeled in its header");
	check(!hasComment(interior, "euclidean") && !hasComment(euclidean, "interior distance"),
		"interior and euclidean exports cannot be confused");
	check(hasComment(interior, "maximum of the data set") && hasComment(euclidean, "fixed maximum"),
		"the header states which normalization was used");
	check(hasComment(interior, "Isolines: every"), "the header states the isoline spacing");

	// Both exports have to stay readable by the vertex color OBJ convention: 6 floats per
	// vertex line.
	std::ifstream stream("distance_map_interior.obj");
	std::string line;
	bool everyVertexLineHasColors = true;
	while (std::getline(stream, line))
	{
		if (line.rfind("v ", 0) != 0)
		{
			continue;
		}

		std::istringstream lineStream(line);
		std::string keyword;
		lineStream >> keyword;
		double value = 0.;
		int valueCount = 0;
		while (lineStream >> value)
		{
			++valueCount;
		}
		everyVertexLineHasColors = everyVertexLineHasColors && valueCount == 6;
	}

	check(everyVertexLineHasColors, "every vertex line carries a position and a color");
}

}

int main()
{
	testGradient();
	testNormalization();
	testContours();
	testOutputFormat();

	std::cout << "\n" << (failures == 0 ? "All checks passed." : std::to_string(failures) + " check(s) failed.") << std::endl;

	return (failures == 0) ? 0 : 1;
}
