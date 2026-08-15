/**
 * Validation harness for the heat-method interior distances and their use in the PMVC
 * weight formula.
 *
 * Phase 1 acceptance: voxelizes a convex cube cage and a non-convex U-shaped cage,
 *   dumps the grids to OBJ and verifies the fill is watertight (no exterior leak into
 *   the enclosed volume, interior volume matches the analytic volume).
 * Phase 2 acceptance: on the convex cage the interior distance must match the Euclidean
 *   distance within voxel-resolution error; on the U-shaped cage the interior distance
 *   between the two arms must be clearly larger than the straight line through the wall.
 * Phase 5 validation: runs the Euclidean (depth-weighted) PMVC formula and the
 *   interior-distance formula through a CPU mirror of the cubemap pipeline on the
 *   same convex cage and reports the max/mean weight deviation. The per-texel math
 *   mirrors the unified PMVCCompute.comp of the Ring pipeline one to one.
 * Phase 6 validation: the n-hit accumulation of the Ring pipeline. Every second hit
 *   carries the negative contributions and is always omitted, except for a hit count of
 *   three where the first, second and third hit are weighted by alpha, beta and theta.
 *   The three-hit weights and the interior distances have to work on their own and
 *   combined.
 */

#include <cagedeformations/InteriorDistance.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
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
// Test cages.
// --------------------------------------------------------------------------------------

struct TestMesh
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
};

/// Axis-aligned cube [0, size]^3 with each face split into subdivisions^2 quads (2 triangles each).
TestMesh makeSubdividedCube(double size, int subdivisions)
{
	TestMesh mesh;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<Eigen::Vector3i> faces;

	auto addFaceGrid = [&](const Eigen::Vector3d& corner, const Eigen::Vector3d& du, const Eigen::Vector3d& dv)
	{
		const int base = static_cast<int>(vertices.size());
		for (int v = 0; v <= subdivisions; ++v)
		{
			for (int u = 0; u <= subdivisions; ++u)
			{
				vertices.push_back(corner + (du * u + dv * v) / subdivisions);
			}
		}
		const int stride = subdivisions + 1;
		for (int v = 0; v < subdivisions; ++v)
		{
			for (int u = 0; u < subdivisions; ++u)
			{
				const int i00 = base + v * stride + u;
				const int i10 = i00 + 1;
				const int i01 = i00 + stride;
				const int i11 = i01 + 1;
				faces.emplace_back(i00, i10, i11);
				faces.emplace_back(i00, i11, i01);
			}
		}
	};

	const Eigen::Vector3d ex(size, 0, 0), ey(0, size, 0), ez(0, 0, size);
	const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
	addFaceGrid(zero, ey, ex);              // z = 0
	addFaceGrid(ez, ex, ey);                // z = size
	addFaceGrid(zero, ex, ez);              // y = 0
	addFaceGrid(ey, ez, ex);                // y = size
	addFaceGrid(zero, ez, ey);              // x = 0
	addFaceGrid(ex, ey, ez);                // x = size

	mesh.V.resize(static_cast<Eigen::Index>(vertices.size()), 3);
	for (size_t i = 0; i < vertices.size(); ++i)
	{
		mesh.V.row(static_cast<Eigen::Index>(i)) = vertices[i];
	}
	mesh.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
	for (size_t i = 0; i < faces.size(); ++i)
	{
		mesh.F.row(static_cast<Eigen::Index>(i)) = faces[i];
	}

	return mesh;
}

/**
 * Watertight U-shaped prism: outline in the xy plane (opening towards +y, arms of width 1,
 * outer size 3 x 3, base height 1), extruded from z = 0 to z = depth.
 */
TestMesh makeUCage(double depth)
{
	// Outline in counter-clockwise order; (3,1) and (0,1) are collinear helpers on the
	// outer walls so the caps can be built from three rectangles.
	const double outline[10][2] = {
		{ 0, 0 }, { 3, 0 }, { 3, 1 }, { 3, 3 }, { 2, 3 }, { 2, 1 }, { 1, 1 }, { 1, 3 }, { 0, 3 }, { 0, 1 }
	};
	constexpr int outlineCount = 10;

	TestMesh mesh;
	mesh.V.resize(2 * outlineCount, 3);
	for (int i = 0; i < outlineCount; ++i)
	{
		mesh.V.row(i) = Eigen::Vector3d(outline[i][0], outline[i][1], 0.0);
		mesh.V.row(outlineCount + i) = Eigen::Vector3d(outline[i][0], outline[i][1], depth);
	}

	std::vector<Eigen::Vector3i> faces;
	// Side walls.
	for (int i = 0; i < outlineCount; ++i)
	{
		const int a = i;
		const int b = (i + 1) % outlineCount;
		faces.emplace_back(a, b, outlineCount + b);
		faces.emplace_back(a, outlineCount + b, outlineCount + a);
	}
	// Caps: base rectangle (0,0)-(3,1), left arm (0,1)-(1,3), right arm (2,1)-(3,3),
	// expressed through outline vertex indices (see the outline table above).
	const int capQuads[3][4] = {
		{ 0, 1, 2, 9 },  // base: (0,0) (3,0) (3,1) (0,1)
		{ 9, 6, 7, 8 },  // left arm: (0,1) (1,1) (1,3) (0,3)
		{ 5, 2, 3, 4 }   // right arm: (2,1) (3,1) (3,3) (2,3)
	};
	for (const auto& quad : capQuads)
	{
		faces.emplace_back(quad[0], quad[1], quad[2]);
		faces.emplace_back(quad[0], quad[2], quad[3]);
		faces.emplace_back(outlineCount + quad[0], outlineCount + quad[2], outlineCount + quad[1]);
		faces.emplace_back(outlineCount + quad[0], outlineCount + quad[3], outlineCount + quad[2]);
	}

	mesh.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
	for (size_t i = 0; i < faces.size(); ++i)
	{
		mesh.F.row(static_cast<Eigen::Index>(i)) = faces[i];
	}

	return mesh;
}

// --------------------------------------------------------------------------------------
// CPU mirror of the cubemap PMVC pipeline (matches the viewer's shader math one to one).
// --------------------------------------------------------------------------------------

struct RayHit
{
	int triangle = -1;
	double rayT = 0.0; // distance along the normalized ray direction
	double b0 = 0.0, b1 = 0.0, b2 = 0.0;
};

bool intersectTriangle(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
	const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, RayHit& hit)
{
	const Eigen::Vector3d e1 = b - a;
	const Eigen::Vector3d e2 = c - a;
	const Eigen::Vector3d p = dir.cross(e2);
	const double det = e1.dot(p);
	if (std::abs(det) < 1e-14)
	{
		return false;
	}
	const double invDet = 1.0 / det;
	const Eigen::Vector3d s = origin - a;
	const double u = s.dot(p) * invDet;
	if (u < -1e-9 || u > 1.0 + 1e-9)
	{
		return false;
	}
	const Eigen::Vector3d q = s.cross(e1);
	const double v = dir.dot(q) * invDet;
	if (v < -1e-9 || u + v > 1.0 + 1e-9)
	{
		return false;
	}
	const double t = e2.dot(q) * invDet;
	if (t <= 1e-9)
	{
		return false;
	}
	hit.rayT = t;
	hit.b0 = 1.0 - u - v;
	hit.b1 = u;
	hit.b2 = v;
	return true;
}

/// Same texel footprint integral as SphereWeightCalculator.
double areaElement(double x, double y)
{
	return std::atan2(x * y, std::sqrt(x * x + y * y + 1.0));
}

double computeSolidAngle(int texelX, int texelY, int faceSize)
{
	const double size = static_cast<double>(faceSize);
	const double u = (2.0 * (texelX + 0.5) / size) - 1.0;
	const double v = (2.0 * (texelY + 0.5) / size) - 1.0;
	const double invResolution = 1.0 / size;
	const double x0 = u - invResolution;
	const double y0 = v - invResolution;
	const double x1 = u + invResolution;
	const double y1 = v + invResolution;
	return areaElement(x0, y0) - areaElement(x0, y1) - areaElement(x1, y0) + areaElement(x1, y1);
}

/// Depth buffer value of a hit at eye-space depth z for a [0, 1] depth range perspective
/// projection (GLM_FORCE_DEPTH_ZERO_TO_ONE), exactly what the cubemap rasterizer writes.
double depthFromEyeZ(double zEye, double nearPlane, double farPlane)
{
	if (zEye <= 0.0)
	{
		return 0.0;
	}
	const double depth = (farPlane * (zEye - nearPlane)) / (zEye * (farPlane - nearPlane));
	return std::clamp(depth, 0.0, 1.0);
}

/// Inverse of depthFromEyeZ, matching the reconstruction in PMVCCompute.comp.
double eyeZFromDepth(double depth, double nearPlane, double farPlane)
{
	return (farPlane * nearPlane) / (farPlane - depth * (farPlane - nearPlane));
}

struct PMVCMirrorParams
{
	int faceSize = 32;
	uint32_t hitCount = 1;
	/// Weights of the three-hit variant, only used when hitCount == 3.
	double alpha = 1.0;
	double beta = -1.0;
	double theta = 1.0;
	/// Offset variant (PMVCO): weight by the solid angle alone, without a distance term.
	bool solidAngleOnly = false;
	/// nullptr = Euclidean variant (weight from rasterized depth); otherwise the
	/// interior detour table (rows = cage vertices, cols = mesh vertices, entries are
	/// interior minus Euclidean distance, >= 0).
	const Eigen::MatrixXf* interiorDetours = nullptr;
};

/**
 * Weight of a hit, mirroring CubemapRenderInstance::HitWeight: the three-hit variant
 * weights the first, second and third hit by alpha, beta and theta, every other
 * configuration keeps the positive hits and omits the negative (every second) ones. A
 * weight of zero means the layer is only rendered so the next one can be peeled.
 */
double hitWeight(const PMVCMirrorParams& params, const uint32_t hitIndex)
{
	if (params.hitCount == 3)
	{
		switch (hitIndex)
		{
		case 0: return params.alpha;
		case 1: return params.beta;
		default: return params.theta;
		}
	}

	return (hitIndex % 2u == 0u) ? 1.0 : 0.0;
}

/// Near/far plane heuristics copied from CubemapRenderInstance::UpdateProjectionPlanes.
void computeProjectionPlanes(const TestMesh& cage, const Eigen::MatrixXd& meshVertices,
	double& nearPlane, double& farPlane)
{
	double maxCageDistance = 0.0;
	for (Eigen::Index i = 0; i < cage.V.rows(); ++i)
	{
		for (Eigen::Index j = i + 1; j < cage.V.rows(); ++j)
		{
			maxCageDistance = std::max(maxCageDistance, (cage.V.row(i) - cage.V.row(j)).norm());
		}
	}
	farPlane = std::max(maxCageDistance, 1e-3);

	double minDistance = std::numeric_limits<double>::max();
	for (Eigen::Index m = 0; m < meshVertices.rows(); ++m)
	{
		const Eigen::Vector3d p = meshVertices.row(m);
		for (Eigen::Index f = 0; f < cage.F.rows(); ++f)
		{
			const Eigen::Vector3d a = cage.V.row(cage.F(f, 0));
			const Eigen::Vector3d b = cage.V.row(cage.F(f, 1));
			const Eigen::Vector3d c = cage.V.row(cage.F(f, 2));
			// Distance point to triangle plane region via projection onto the triangle.
			const Eigen::Vector3d ab = b - a, ac = c - a, ap = p - a;
			const double d1 = ab.dot(ap), d2 = ac.dot(ap);
			Eigen::Vector3d closest;
			if (d1 <= 0 && d2 <= 0) { closest = a; }
			else
			{
				const Eigen::Vector3d bp = p - b;
				const double d3 = ab.dot(bp), d4 = ac.dot(bp);
				if (d3 >= 0 && d4 <= d3) { closest = b; }
				else
				{
					const double vc = d1 * d4 - d3 * d2;
					if (vc <= 0 && d1 >= 0 && d3 <= 0) { closest = a + (d1 / (d1 - d3)) * ab; }
					else
					{
						const Eigen::Vector3d cp = p - c;
						const double d5 = ab.dot(cp), d6 = ac.dot(cp);
						if (d6 >= 0 && d5 <= d6) { closest = c; }
						else
						{
							const double vb = d5 * d2 - d1 * d6;
							if (vb <= 0 && d2 >= 0 && d6 <= 0) { closest = a + (d2 / (d2 - d6)) * ac; }
							else
							{
								const double va = d3 * d6 - d5 * d4;
								if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
								{
									closest = b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);
								}
								else
								{
									const Eigen::Vector3d n = ab.cross(ac).normalized();
									closest = p - n.dot(ap) * n;
								}
							}
						}
					}
				}
			}
			minDistance = std::min(minDistance, (p - closest).norm());
		}
	}

	nearPlane = std::max(minDistance, 1e-4);
	if (nearPlane >= farPlane)
	{
		nearPlane = std::max(farPlane * 0.001, 1e-4);
	}
}

/**
 * PMVC coordinates for every mesh vertex through the cubemap formulation. Each texel of a
 * 6 x faceSize^2 cubemap around the mesh vertex is ray cast against the cage; hit pass k
 * uses the k-th nearest intersection (the CPU equivalent of the depth peeling passes of
 * the Ring pipeline). Per texel and pass the weight is
 *     w = solidAngle * (1 - depth) * hitWeight
 * where depth is the rasterized depth value for the Euclidean variant; the
 * interior-distance variant lengthens the rasterized hit distance by the barycentric
 * interpolation of the three cage vertex interior detours before re-encoding, so it
 * reduces exactly to the Euclidean variant wherever the detours vanish. The offset
 * variant drops the distance term and weights by the solid angle alone.
 */
Eigen::MatrixXd computePMVCMirror(const TestMesh& cage, const Eigen::MatrixXd& meshVertices,
	const PMVCMirrorParams& params)
{
	const int faceSize = params.faceSize;
	double nearPlane = 0.0, farPlane = 1.0;
	computeProjectionPlanes(cage, meshVertices, nearPlane, farPlane);

	// Face bases: forward, right, up (any orthonormal set covering the sphere works and
	// matches the coverage of the viewer's six 90 degree frusta).
	const Eigen::Vector3d bases[6][3] = {
		{ {  1, 0, 0 }, { 0, 0, -1 }, { 0, -1, 0 } },
		{ { -1, 0, 0 }, { 0, 0,  1 }, { 0, -1, 0 } },
		{ { 0,  1, 0 }, { 1, 0, 0 }, { 0, 0,  1 } },
		{ { 0, -1, 0 }, { 1, 0, 0 }, { 0, 0, -1 } },
		{ { 0, 0,  1 }, { 1, 0, 0 }, { 0, -1, 0 } },
		{ { 0, 0, -1 }, { -1, 0, 0 }, { 0, -1, 0 } }
	};

	Eigen::MatrixXd lambda(meshVertices.rows(), cage.V.rows());
	lambda.setZero();

	for (Eigen::Index meshIdx = 0; meshIdx < meshVertices.rows(); ++meshIdx)
	{
		const Eigen::Vector3d center = meshVertices.row(meshIdx);
		double wsum = 0.0;

		for (int face = 0; face < 6; ++face)
		{
			for (int y = 0; y < faceSize; ++y)
			{
				for (int x = 0; x < faceSize; ++x)
				{
					const double u = (2.0 * (x + 0.5) / faceSize) - 1.0;
					const double v = (2.0 * (y + 0.5) / faceSize) - 1.0;
					const Eigen::Vector3d dir = (bases[face][0] + u * bases[face][1] + v * bases[face][2]).normalized();
					const double cosTheta = 1.0 / std::sqrt(1.0 + u * u + v * v);
					const double solidAngle = computeSolidAngle(x, y, faceSize);

					// All intersections along the ray, nearest first (depth peel order).
					std::vector<RayHit> hits;
					for (Eigen::Index f = 0; f < cage.F.rows(); ++f)
					{
						RayHit hit;
						if (intersectTriangle(center, dir,
							cage.V.row(cage.F(f, 0)), cage.V.row(cage.F(f, 1)), cage.V.row(cage.F(f, 2)), hit))
						{
							hit.triangle = static_cast<int>(f);
							hits.push_back(hit);
						}
					}
					std::sort(hits.begin(), hits.end(), [](const RayHit& a, const RayHit& b) { return a.rayT < b.rayT; });

					for (uint32_t pass = 0; pass < params.hitCount && pass < hits.size(); ++pass)
					{
						const double weightScale = hitWeight(params, pass);
						if (weightScale == 0.0)
						{
							// Rendered only so the next layer can be peeled against it.
							continue;
						}

						const RayHit& hit = hits[pass];
						const double rasterDepth = depthFromEyeZ(hit.rayT * cosTheta, nearPlane, farPlane);
						if (rasterDepth >= 0.999999)
						{
							continue; // matches the "no hit" depth test in the shader
						}

						double depth = rasterDepth;
						if (params.interiorDetours != nullptr && !params.solidAngleOnly)
						{
							const int i0 = cage.F(hit.triangle, 0);
							const int i1 = cage.F(hit.triangle, 1);
							const int i2 = cage.F(hit.triangle, 2);
							const double detour =
								hit.b0 * (*params.interiorDetours)(i0, meshIdx) +
								hit.b1 * (*params.interiorDetours)(i1, meshIdx) +
								hit.b2 * (*params.interiorDetours)(i2, meshIdx);
							const double zEye = eyeZFromDepth(rasterDepth, nearPlane, farPlane);
							depth = depthFromEyeZ(zEye + std::max(detour, 0.0) * cosTheta, nearPlane, farPlane);
						}

						double w = params.solidAngleOnly ? solidAngle : solidAngle * (1.0 - depth);
						if (w <= 0.0)
						{
							continue;
						}

						// The hit weight is applied last so a negative beta flips the sign
						// of the whole contribution instead of dropping it above.
						w *= weightScale;

						lambda(meshIdx, cage.F(hit.triangle, 0)) += hit.b0 * w;
						lambda(meshIdx, cage.F(hit.triangle, 1)) += hit.b1 * w;
						lambda(meshIdx, cage.F(hit.triangle, 2)) += hit.b2 * w;
						wsum += w;
					}
				}
			}
		}

		if (std::abs(wsum) > std::numeric_limits<double>::epsilon())
		{
			lambda.row(meshIdx) /= wsum;
		}
	}

	return lambda;
}

Eigen::MatrixXf euclideanDistanceTable(const TestMesh& cage, const Eigen::MatrixXd& meshVertices)
{
	Eigen::MatrixXf table(cage.V.rows(), meshVertices.rows());
	for (Eigen::Index c = 0; c < cage.V.rows(); ++c)
	{
		for (Eigen::Index m = 0; m < meshVertices.rows(); ++m)
		{
			table(c, m) = static_cast<float>((cage.V.row(c) - meshVertices.row(m)).norm());
		}
	}
	return table;
}

// --------------------------------------------------------------------------------------
// Phases.
// --------------------------------------------------------------------------------------

void testPhase1Voxelization(const std::string& outputDirectory)
{
	std::cout << "\n=== Phase 1: voxelization ===" << std::endl;

	const TestMesh cube = makeSubdividedCube(2.0, 1);
	const int resolution = 48;
	const VoxelGrid cubeGrid = voxelizeCageInterior(cube.V, cube.F, resolution);

	check(!cubeGrid.labels.empty(), "cube: voxel grid created");

	size_t interior = 0, boundary = 0, exterior = 0;
	for (const uint8_t label : cubeGrid.labels)
	{
		interior += label == VOXEL_INTERIOR;
		boundary += label == VOXEL_BOUNDARY;
		exterior += label == VOXEL_EXTERIOR;
	}
	std::cout << "cube grid " << cubeGrid.Nx << "x" << cubeGrid.Ny << "x" << cubeGrid.Nz
		<< ": interior=" << interior << " boundary=" << boundary << " exterior=" << exterior << std::endl;

	// Watertight fill: the enclosed volume (interior + boundary voxels) must bracket the
	// analytic cube volume and no interior voxel may touch an exterior voxel.
	const double h = cubeGrid.voxelSize;
	const double solidVolume = static_cast<double>(interior + boundary) * h * h * h;
	const double interiorVolume = static_cast<double>(interior) * h * h * h;
	const double analytic = 2.0 * 2.0 * 2.0;
	std::cout << "cube volume: analytic=" << analytic << " interiorVoxels=" << interiorVolume
		<< " interior+boundary=" << solidVolume << std::endl;
	check(interiorVolume < analytic && solidVolume > analytic, "cube: voxel volume brackets the analytic volume");

	bool leak = false;
	static constexpr int offsets[6][3] = { { -1, 0, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { 0, 1, 0 }, { 0, 0, -1 }, { 0, 0, 1 } };
	for (int z = 0; z < cubeGrid.Nz && !leak; ++z)
	{
		for (int y = 0; y < cubeGrid.Ny && !leak; ++y)
		{
			for (int x = 0; x < cubeGrid.Nx && !leak; ++x)
			{
				if (cubeGrid.label(x, y, z) != VOXEL_INTERIOR)
				{
					continue;
				}
				for (const auto& offset : offsets)
				{
					const int nx = x + offset[0], ny = y + offset[1], nz = z + offset[2];
					if (cubeGrid.inBounds(nx, ny, nz) && cubeGrid.label(nx, ny, nz) == VOXEL_EXTERIOR)
					{
						leak = true;
					}
				}
			}
		}
	}
	check(!leak, "cube: no interior voxel touches an exterior voxel (watertight boundary shell)");

	const TestMesh uCage = makeUCage(1.0);
	const VoxelGrid uGrid = voxelizeCageInterior(uCage.V, uCage.F, resolution);
	size_t uInterior = 0, uBoundary = 0;
	for (const uint8_t label : uGrid.labels)
	{
		uInterior += label == VOXEL_INTERIOR;
		uBoundary += label == VOXEL_BOUNDARY;
	}
	const double uh = uGrid.voxelSize;
	const double uInteriorVolume = static_cast<double>(uInterior) * uh * uh * uh;
	const double uSolidVolume = static_cast<double>(uInterior + uBoundary) * uh * uh * uh;
	const double uAnalytic = 7.0; // base 3x1x1 plus two arms 1x2x1
	std::cout << "U cage volume: analytic=" << uAnalytic << " interiorVoxels=" << uInteriorVolume
		<< " interior+boundary=" << uSolidVolume << std::endl;
	check(uInteriorVolume < uAnalytic && uSolidVolume > uAnalytic, "U cage: voxel volume brackets the analytic volume");

	// The gap between the arms (around x = 1.5, y = 2) must stay exterior: interior
	// distance may not leak across the opening.
	const Eigen::Vector3f gapPoint(1.5f, 2.0f, 0.5f);
	const Eigen::Vector3f gapLocal = (gapPoint - uGrid.origin) / uGrid.voxelSize;
	const int gapX = static_cast<int>(gapLocal.x());
	const int gapY = static_cast<int>(gapLocal.y());
	const int gapZ = static_cast<int>(gapLocal.z());
	check(uGrid.inBounds(gapX, gapY, gapZ) && uGrid.label(gapX, gapY, gapZ) == VOXEL_EXTERIOR,
		"U cage: the gap between the arms is classified exterior");

	writeVoxelGridOBJ(outputDirectory + "/voxelgrid_cube.obj", cubeGrid);
	writeVoxelGridOBJ(outputDirectory + "/voxelgrid_ucage.obj", uGrid);
	std::cout << "voxel grids dumped to " << outputDirectory << "/voxelgrid_{cube,ucage}.obj" << std::endl;
}

void testPhase2InteriorDistance()
{
	std::cout << "\n=== Phase 2: heat-method interior distance ===" << std::endl;

	InteriorDistanceParams params;
	params.resolution = 48;

	// Convex cage: interior distance must match Euclidean distance within voxel error.
	const TestMesh cube = makeSubdividedCube(2.0, 2);
	Eigen::MatrixXd samples(5, 3);
	samples << 1.0, 1.0, 1.0,
		0.5, 0.5, 0.5,
		1.5, 1.4, 0.6,
		0.3, 1.7, 1.0,
		1.0, 0.4, 1.6;

	Eigen::MatrixXf interior;
	computeInteriorDistances(cube.V, cube.F, samples, params, interior);

	const double voxelSize = 2.0 / params.resolution;
	double maxAbsError = 0.0, meanAbsError = 0.0;
	for (Eigen::Index c = 0; c < cube.V.rows(); ++c)
	{
		for (Eigen::Index m = 0; m < samples.rows(); ++m)
		{
			const double euclidean = (cube.V.row(c) - samples.row(m)).norm();
			const double error = std::abs(interior(c, m) - euclidean);
			maxAbsError = std::max(maxAbsError, error);
			meanAbsError += error;
		}
	}
	meanAbsError /= static_cast<double>(cube.V.rows() * samples.rows());
	std::cout << "convex cube: |interior - euclidean| max=" << maxAbsError << " mean=" << meanAbsError
		<< " (voxel size " << voxelSize << ")" << std::endl;
	check(maxAbsError <= 4.0 * voxelSize, "convex cage: interior distance matches Euclidean within voxel-resolution error");

	bool allFinite = interior.allFinite() && (interior.array() >= 0.f).all();
	check(allFinite, "convex cage: table is finite and non-negative");

	// Non-convex cage: distance from the tip of the left arm to the tip of the right arm
	// must follow the U (down the arm, across the base, up again), not the straight line.
	const TestMesh uCage = makeUCage(1.0);
	Eigen::MatrixXd uSamples(2, 3);
	uSamples << 2.5, 2.5, 0.5,  // inside right arm, near the top
		0.5, 2.5, 0.5;          // inside left arm, near the top
	Eigen::MatrixXf uInterior;
	computeInteriorDistances(uCage.V, uCage.F, uSamples, params, uInterior);

	// Cage vertex 7 = (1, 3, 0): top of the left arm's inner wall.
	const int leftArmVertex = 7;
	const double euclidCross = (uCage.V.row(leftArmVertex) - uSamples.row(0)).norm();
	const double interiorCross = uInterior(leftArmVertex, 0);
	// Shortest interior path: down the left arm, across the base, up the right arm (roughly
	// 2 + 1.5 + 2 = 5.5 in this geometry, against a Euclidean distance of about 1.6).
	std::cout << "U cage: cross-arm euclidean=" << euclidCross << " interior=" << interiorCross << std::endl;
	check(interiorCross > 2.0 * euclidCross, "non-convex cage: interior distance is much larger than Euclidean across the gap");

	const double euclidSameArm = (uCage.V.row(leftArmVertex) - uSamples.row(1)).norm();
	const double interiorSameArm = uInterior(leftArmVertex, 1);
	std::cout << "U cage: same-arm euclidean=" << euclidSameArm << " interior=" << interiorSameArm << std::endl;
	check(std::abs(interiorSameArm - euclidSameArm) < 6.0 * uCage.V.col(0).maxCoeff() / params.resolution,
		"non-convex cage: same-arm interior distance stays close to Euclidean");
}

void testPhase5WeightParity()
{
	std::cout << "\n=== Phase 5: Euclidean vs interior-distance PMVC weight diff (convex cage) ===" << std::endl;

	// Deliberately coarse convex cage (12 triangles): the detour formulation must match
	// the Euclidean variant here too, unlike interpolating absolute per-vertex distances
	// which overestimates the hit distance mid-triangle on coarse cages.
	const TestMesh cube = makeSubdividedCube(2.0, 1);
	Eigen::MatrixXd samples(4, 3);
	samples << 1.0, 1.0, 1.0,
		0.6, 0.7, 0.8,
		1.4, 1.3, 0.7,
		0.8, 1.5, 1.2;

	PMVCMirrorParams euclidParams;
	euclidParams.faceSize = 32;
	const Eigen::MatrixXd euclidWeights = computePMVCMirror(cube, samples, euclidParams);

	// With exact distances the detours are all zero and the variant must reduce exactly
	// to the Euclidean weights.
	const Eigen::MatrixXf analyticTable = euclideanDistanceTable(cube, samples);
	const Eigen::MatrixXf zeroDetours = (analyticTable - analyticTable).cwiseMax(0.f);
	PMVCMirrorParams analyticParams = euclidParams;
	analyticParams.interiorDetours = &zeroDetours;
	const Eigen::MatrixXd analyticWeights = computePMVCMirror(cube, samples, analyticParams);

	const double analyticMax = (analyticWeights - euclidWeights).cwiseAbs().maxCoeff();
	std::cout << "zero-detour variant (must reduce to Euclidean exactly): max=" << analyticMax << std::endl;
	check(analyticMax < 1e-12, "zero-detour interior variant reduces exactly to Euclidean PMVC weights");

	// Then with the actual heat-method detours: only the clamped voxel-scale solver
	// noise remains on a convex cage.
	InteriorDistanceParams distanceParams;
	distanceParams.resolution = 48;
	Eigen::MatrixXf heatTable;
	computeInteriorDistances(cube.V, cube.F, samples, distanceParams, heatTable);
	const Eigen::MatrixXf heatDetours = (heatTable - analyticTable).cwiseMax(0.f);

	std::cout << "heat-method detours on the convex cage: max=" << heatDetours.maxCoeff()
		<< " (voxel size " << 2.0 / distanceParams.resolution << ")" << std::endl;

	PMVCMirrorParams heatParams = euclidParams;
	heatParams.interiorDetours = &heatDetours;
	const Eigen::MatrixXd heatWeights = computePMVCMirror(cube, samples, heatParams);

	const double heatMaxDeviation = (heatWeights - euclidWeights).cwiseAbs().maxCoeff();
	const double heatMeanDeviation = (heatWeights - euclidWeights).cwiseAbs().mean();
	std::cout << "heat-method detours: weight deviation max=" << heatMaxDeviation << " mean=" << heatMeanDeviation << std::endl;
	check(heatMaxDeviation < 3e-2, "interior-distance PMVC matches Euclidean PMVC on a coarse convex cage (voxel-scale deviation)");
	check(heatWeights.allFinite(), "interior-distance weights are finite");

	double rowSumError = 0.0;
	for (Eigen::Index row = 0; row < heatWeights.rows(); ++row)
	{
		rowSumError = std::max(rowSumError, std::abs(heatWeights.row(row).sum() - 1.0));
	}
	check(rowSumError < 1e-9, "interior-distance weights are normalized (rows sum to 1)");
}

void testPhase6MultiHit()
{
	std::cout << "\n=== Phase 6: n-hit PMVC through the Ring pipeline ===" << std::endl;

	const TestMesh uCage = makeUCage(1.0);
	Eigen::MatrixXd samples(3, 3);
	samples << 0.5, 2.0, 0.5,   // left arm: rays towards +x cross the gap and hit the right arm (3 hits)
		1.5, 0.5, 0.5,          // base
		2.5, 2.0, 0.5;          // right arm

	InteriorDistanceParams distanceParams;
	distanceParams.resolution = 48;
	Eigen::MatrixXf table;
	computeInteriorDistances(uCage.V, uCage.F, samples, distanceParams, table);
	const Eigen::MatrixXf detours = (table - euclideanDistanceTable(uCage, samples)).cwiseMax(0.f);
	const Eigen::MatrixXf zeroDetours = Eigen::MatrixXf::Zero(detours.rows(), detours.cols());

	const auto isNormalized = [](const Eigen::MatrixXd& weights)
	{
		double rowSumError = 0.0;
		for (Eigen::Index row = 0; row < weights.rows(); ++row)
		{
			rowSumError = std::max(rowSumError, std::abs(weights.row(row).sum() - 1.0));
		}
		return rowSumError < 1e-9;
	};

	PMVCMirrorParams singleHit;
	singleHit.faceSize = 16;
	singleHit.hitCount = 1;
	const Eigen::MatrixXd single = computePMVCMirror(uCage, samples, singleHit);

	// The negative contributions of every second hit are always omitted outside the
	// three-hit variant, so an even hit count must match the odd one below it.
	PMVCMirrorParams twoHits = singleHit;
	twoHits.hitCount = 2;
	const Eigen::MatrixXd two = computePMVCMirror(uCage, samples, twoHits);
	check((two - single).cwiseAbs().maxCoeff() == 0.0, "hitCount=2 omits the negative hit and equals hitCount=1");

	PMVCMirrorParams fourHits = singleHit;
	fourHits.hitCount = 4;
	const Eigen::MatrixXd four = computePMVCMirror(uCage, samples, fourHits);
	check((four - single).cwiseAbs().maxCoeff() > 0.0, "hitCount=4 adds the third hit on the non-convex cage");

	// A hit count of exactly three switches to the alpha / beta / theta weights. With
	// beta = 0 that is the same set of hits an even hit count accumulates.
	PMVCMirrorParams threeHitNoBeta = singleHit;
	threeHitNoBeta.hitCount = 3;
	threeHitNoBeta.alpha = 1.0;
	threeHitNoBeta.beta = 0.0;
	threeHitNoBeta.theta = 1.0;
	const Eigen::MatrixXd threeNoBeta = computePMVCMirror(uCage, samples, threeHitNoBeta);
	check((threeNoBeta - four).cwiseAbs().maxCoeff() == 0.0,
		"three-hit weights with beta=0 accumulate the same hits as the omitted-negative variant");

	// The default three-hit variant subtracts the second hit.
	PMVCMirrorParams threeHit = singleHit;
	threeHit.hitCount = 3;
	const Eigen::MatrixXd three = computePMVCMirror(uCage, samples, threeHit);
	check(three.allFinite(), "3-hit weights are finite (no NaN)");
	check(isNormalized(three), "3-hit weights are normalized");
	check((three - threeNoBeta).cwiseAbs().maxCoeff() > 0.0, "the negative second hit changes the 3-hit weights");

	// Interior distances on their own.
	PMVCMirrorParams interiorSingle = singleHit;
	interiorSingle.interiorDetours = &detours;
	const Eigen::MatrixXd interior = computePMVCMirror(uCage, samples, interiorSingle);
	check(interior.allFinite(), "interior-distance weights are finite");
	check(isNormalized(interior), "interior-distance weights are normalized");
	check((interior - single).cwiseAbs().maxCoeff() > 0.0, "interior distances change the weights on the non-convex cage");

	// Interior distances combined with the three-hit weights: both features have to work
	// together in the one pipeline.
	PMVCMirrorParams interiorThreeHit = threeHit;
	interiorThreeHit.interiorDetours = &detours;
	const Eigen::MatrixXd interiorThree = computePMVCMirror(uCage, samples, interiorThreeHit);
	check(interiorThree.allFinite(), "combined 3-hit + interior-distance weights are finite");
	check(isNormalized(interiorThree), "combined 3-hit + interior-distance weights are normalized");
	check((interiorThree - three).cwiseAbs().maxCoeff() > 0.0,
		"interior distances still change the weights of the 3-hit variant");
	check((interiorThree - interior).cwiseAbs().maxCoeff() > 0.0,
		"the 3-hit weights still change the interior-distance variant");

	// Zero detours must reduce the combined variant exactly to the Euclidean 3-hit one.
	PMVCMirrorParams zeroDetourThreeHit = threeHit;
	zeroDetourThreeHit.interiorDetours = &zeroDetours;
	const Eigen::MatrixXd zeroDetourThree = computePMVCMirror(uCage, samples, zeroDetourThreeHit);
	check((zeroDetourThree - three).cwiseAbs().maxCoeff() < 1e-12,
		"zero detours reduce the combined variant exactly to the Euclidean 3-hit weights");

	const double multiHitInfluence = (three - single).cwiseAbs().maxCoeff();
	std::cout << "difference single-hit vs 3-hit on the U cage: " << multiHitInfluence << std::endl;
	check(multiHitInfluence > 0.0, "later hit passes contribute on the non-convex cage");
}

void testPhase7OffsetVariant()
{
	std::cout << "\n=== Phase 7: offset variant (PMVCO) ===" << std::endl;

	const TestMesh uCage = makeUCage(1.0);
	Eigen::MatrixXd samples(2, 3);
	samples << 0.5, 2.0, 0.5,
		1.5, 0.5, 0.5;

	InteriorDistanceParams distanceParams;
	distanceParams.resolution = 48;
	Eigen::MatrixXf table;
	computeInteriorDistances(uCage.V, uCage.F, samples, distanceParams, table);
	const Eigen::MatrixXf detours = (table - euclideanDistanceTable(uCage, samples)).cwiseMax(0.f);

	PMVCMirrorParams offset;
	offset.faceSize = 16;
	offset.hitCount = 1;
	offset.solidAngleOnly = true;
	const Eigen::MatrixXd offsetWeights = computePMVCMirror(uCage, samples, offset);

	check(offsetWeights.allFinite(), "offset weights are finite");
	check((offsetWeights.array() >= -1e-12).all(), "offset weights are non-negative");

	double rowSumError = 0.0;
	for (Eigen::Index row = 0; row < offsetWeights.rows(); ++row)
	{
		rowSumError = std::max(rowSumError, std::abs(offsetWeights.row(row).sum() - 1.0));
	}
	check(rowSumError < 1e-9, "offset weights are normalized");

	// The offset variant has no distance term, so an interior detour table cannot change
	// its result.
	PMVCMirrorParams offsetWithDetours = offset;
	offsetWithDetours.interiorDetours = &detours;
	const Eigen::MatrixXd offsetInterior = computePMVCMirror(uCage, samples, offsetWithDetours);
	check((offsetInterior - offsetWeights).cwiseAbs().maxCoeff() == 0.0,
		"interior distances do not affect the offset variant");

	PMVCMirrorParams euclidean = offset;
	euclidean.solidAngleOnly = false;
	const Eigen::MatrixXd euclideanWeights = computePMVCMirror(uCage, samples, euclidean);
	check((offsetWeights - euclideanWeights).cwiseAbs().maxCoeff() > 0.0,
		"the offset weighting differs from the distance-weighted variant");
}

} // namespace

int main(int argc, char** argv)
{
	const std::string outputDirectory = argc > 1 ? argv[1] : ".";

	testPhase1Voxelization(outputDirectory);
	testPhase2InteriorDistance();
	testPhase5WeightParity();
	testPhase6MultiHit();
	testPhase7OffsetVariant();

	std::cout << "\n" << (failures == 0 ? "ALL CHECKS PASSED" : "CHECKS FAILED") << " (" << failures << " failures)" << std::endl;
	return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
