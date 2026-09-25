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
 * Phase 5 validation: runs the Euclidean PMVC formula and the interior-distance formula
 *   through a CPU mirror of the cubemap pipeline on the same convex cage and reports the
 *   max/mean weight deviation. The per-texel math mirrors PMVCCompute.comp of the Ring
 *   pipeline one to one.
 * Phase 6 validation: the n-hit accumulation of the Ring pipeline. The hits along a ray
 *   are weighted against each other and normalized to sum to one, so that every ray
 *   carries the same leverage; this checks that the result stays positive, normalized and
 *   reproduces the rest pose for every variant, that skipping the entry hits and the
 *   interior distances preserve all of it, and that the interior distances cannot act on
 *   the first hit, where the detour is zero by construction.
 * Phase 7 validation: the offset variant (PMVCO) weights by the solid angle alone and
 *   ignores the interior detours.
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
	/// Offset variant (PMVCO): weight by the solid angle alone, without a distance term.
	bool solidAngleOnly = false;
	/// Drop the entry (every second) hits from the per-ray weight split.
	bool skipEvenHits = false;
	/// Reproduces the pre-gating weighting solidAngle * (1 - depth) on the first hit only,
	/// which is what the pipeline used before the per-ray normalization. Kept so the tests
	/// can show what it costs: (1 - depth) is proportional to cos(theta)/r - 1/far rather
	/// than to 1/r, so the leverage of a ray depends on its direction.
	bool legacyDepthWeight = false;
	/// nullptr = Euclidean variant; otherwise the interior detour table (rows = cage
	/// vertices, cols = mesh vertices, entries are interior minus Euclidean distance).
	const Eigen::MatrixXf* interiorDetours = nullptr;
};

/**
 * Unnormalized weight of one hit, mirroring hitWeight() of PMVCCompute.comp. rendered is
 * the number of hits the pipeline actually has, so the last of them is treated as leaving
 * towards infinity exactly like the shader does.
 */
double hitWeight(const PMVCMirrorParams& params, const size_t hit, const size_t rendered,
	const std::vector<RayHit>& hits, const TestMesh& cage, const Eigen::Index meshIdx)
{
	if (params.skipEvenHits && (hit % 2) == 1)
	{
		return 0.0;
	}

	const double rCur = hits[hit].rayT;
	const double rPrev = (hit == 0) ? 0.0 : hits[hit - 1].rayT;
	const double rNext = (hit + 1 < rendered) ? hits[hit + 1].rayT : -1.0;

	// Vanish as this hit annihilates with the surface in front of or behind it.
	const double gatePrev = 1.0 - rPrev / rCur;
	const double gateNext = (rNext > 0.0) ? (1.0 / rCur - 1.0 / rNext) : (1.0 / rCur);

	double eta = 1.0;
	if (hit > 0 && params.interiorDetours != nullptr)
	{
		const RayHit& h = hits[hit];
		const double detour = std::max(
			h.b0 * static_cast<double>((*params.interiorDetours)(cage.F(h.triangle, 0), meshIdx)) +
			h.b1 * static_cast<double>((*params.interiorDetours)(cage.F(h.triangle, 1), meshIdx)) +
			h.b2 * static_cast<double>((*params.interiorDetours)(cage.F(h.triangle, 2), meshIdx)), 0.0);
		eta = rCur / std::max(rCur + detour, 1e-20);
	}

	return std::max(gatePrev, 0.0) * std::max(gateNext, 0.0) * eta;
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
 * 6 x faceSize^2 cubemap around the mesh vertex is ray cast against the cage and every
 * intersection within the rendered layers is kept, which is the CPU equivalent of the
 * depth peeling passes of the Ring pipeline.
 *
 * Per texel the hits are first weighted against each other and normalized to sum to one,
 * and only then divided by the Euclidean hit distance:
 *     t_k = w_k / sum_j w_j,   a_k = t_k / r_k * solidAngle
 * so every ray contributes the same leverage sum_k a_k r_k = 1 no matter how often it
 * crosses the cage. That is what makes sum_c lambda_c p_c == x hold.
 *
 * Unlike the shader this works on the ray parameter directly instead of unprojecting the
 * rasterized depth, so it is the exact form of the same formula; only the far plane
 * clipping of the rasterizer is reproduced, because it decides how many hits exist.
 */
struct PMVCMirrorResult
{
	/// Accumulated, not yet normalized weights (rows = mesh vertices).
	Eigen::MatrixXd lambda;
	/// Accumulated weight sum per mesh vertex, the normalization denominator.
	Eigen::VectorXd wsum;
};

PMVCMirrorResult computePMVCMirrorRaw(const TestMesh& cage, const Eigen::MatrixXd& meshVertices,
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

	PMVCMirrorResult result;
	result.lambda.setZero(meshVertices.rows(), cage.V.rows());
	result.wsum.setZero(meshVertices.rows());
	Eigen::MatrixXd& lambda = result.lambda;

	for (Eigen::Index meshIdx = 0; meshIdx < meshVertices.rows(); ++meshIdx)
	{
		const Eigen::Vector3d center = meshVertices.row(meshIdx);
		double wsum = 0.0;

		const auto accumulate = [&](const RayHit& hit, const double weight)
		{
			lambda(meshIdx, cage.F(hit.triangle, 0)) += hit.b0 * weight;
			lambda(meshIdx, cage.F(hit.triangle, 1)) += hit.b1 * weight;
			lambda(meshIdx, cage.F(hit.triangle, 2)) += hit.b2 * weight;
			wsum += weight;
		};

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

					// A ray grazing an edge intersects both triangles sharing it at the same
					// ray parameter. That is one surface crossing, and the rasterizer reports
					// it once (a pixel centre lies in exactly one triangle under the fill
					// rules), so the duplicates are dropped here too.
					hits.erase(std::unique(hits.begin(), hits.end(),
						[](const RayHit& a, const RayHit& b)
						{
							return std::abs(a.rayT - b.rayT) <= 1e-9 * std::max(1.0, std::abs(b.rayT));
						}), hits.end());

					// Only the hits the pipeline actually renders exist: everything the
					// rasterizer clips against the far plane, and everything past the last
					// peeling layer, is not there.
					size_t rendered = 0;
					while (rendered < hits.size() && rendered < params.hitCount &&
						depthFromEyeZ(hits[rendered].rayT * cosTheta, nearPlane, farPlane) < 0.999999)
					{
						++rendered;
					}

					if (rendered == 0)
					{
						continue;
					}

					// The offset variant has no distance term to split along the ray.
					if (params.solidAngleOnly)
					{
						accumulate(hits[0], solidAngle);
						continue;
					}

					if (params.legacyDepthWeight)
					{
						const double depth = depthFromEyeZ(hits[0].rayT * cosTheta, nearPlane, farPlane);
						const double w = solidAngle * (1.0 - depth);
						if (w > 0.0)
						{
							accumulate(hits[0], w);
						}
						continue;
					}

					double weightSum = 0.0;
					for (size_t hit = 0; hit < rendered; ++hit)
					{
						weightSum += hitWeight(params, hit, rendered, hits, cage, meshIdx);
					}

					// Every gate vanished, which only happens when the hits of this ray are
					// coincident. Fall back to the first hit so the ray still carries its
					// leverage: dropping the texel would break the direction symmetry that
					// linear reproduction relies on.
					if (weightSum <= 0.0)
					{
						accumulate(hits[0], solidAngle / hits[0].rayT);
						continue;
					}

					for (size_t hit = 0; hit < rendered; ++hit)
					{
						const double w = hitWeight(params, hit, rendered, hits, cage, meshIdx);
						if (w <= 0.0)
						{
							continue;
						}

						accumulate(hits[hit], (w / weightSum) / hits[hit].rayT * solidAngle);
					}
				}
			}
		}

		result.wsum(meshIdx) = wsum;
	}

	return result;
}

/// Normalized weights of the mirror, the form the viewer's Readback() returns.
Eigen::MatrixXd normalizeMirrorResult(const PMVCMirrorResult& result)
{
	Eigen::MatrixXd lambda = result.lambda;
	for (Eigen::Index meshIdx = 0; meshIdx < lambda.rows(); ++meshIdx)
	{
		const double wsum = result.wsum(meshIdx);
		if (std::abs(wsum) > std::numeric_limits<double>::epsilon())
		{
			lambda.row(meshIdx) /= wsum;
		}
	}

	return lambda;
}

Eigen::MatrixXd computePMVCMirror(const TestMesh& cage, const Eigen::MatrixXd& meshVertices,
	const PMVCMirrorParams& params)
{
	return normalizeMirrorResult(computePMVCMirrorRaw(cage, meshVertices, params));
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

/// Maximum relative error of sum_c lambda_c * p_c against the mesh vertex it belongs to.
double linearReproductionResidual(const Eigen::MatrixXd& weights, const TestMesh& cage,
	const Eigen::MatrixXd& meshVertices)
{
	const Eigen::Vector3d extent =
		cage.V.colwise().maxCoeff().transpose() - cage.V.colwise().minCoeff().transpose();
	const double scale = std::max(extent.norm(), 1e-12);

	double maxResidual = 0.0;
	for (Eigen::Index mesh = 0; mesh < weights.rows(); ++mesh)
	{
		const Eigen::RowVector3d reproduced = weights.row(mesh) * cage.V;
		maxResidual = std::max(maxResidual, (reproduced - meshVertices.row(mesh)).norm() / scale);
	}

	return maxResidual;
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

	PMVCMirrorParams fiveHits = singleHit;
	fiveHits.hitCount = 5;
	const Eigen::MatrixXd five = computePMVCMirror(uCage, samples, fiveHits);
	check((five - single).cwiseAbs().maxCoeff() > 0.0, "the hits beyond the first contribute on the non-convex cage");
	check(five.allFinite(), "multi-hit weights are finite");
	check(isNormalized(five), "multi-hit weights are normalized");

	// Positivity: every weight is a product of non-negative gates divided by a positive
	// distance, so nothing can turn negative however many hits a ray has.
	check((single.array() >= 0.0).all(), "single-hit weights are non-negative");
	check((five.array() >= 0.0).all(), "multi-hit weights are non-negative");

	// Linear reproduction is the property the per-ray normalization exists for: every ray
	// carries the same leverage, so the direction sum cancels and the rest pose comes back
	// exactly. The residual is bounded by the cubemap discretization, not by the algebra.
	const double singleResidual = linearReproductionResidual(single, uCage, samples);
	const double fiveResidual = linearReproductionResidual(five, uCage, samples);
	std::cout << "linear reproduction residual: 1 hit=" << singleResidual
		<< " 5 hits=" << fiveResidual << std::endl;
	check(singleResidual < 1e-12, "single-hit weights reproduce the rest pose exactly");
	check(fiveResidual < 1e-12, "multi-hit weights reproduce the rest pose exactly");

	// The weighting the pipeline used before: solidAngle * (1 - depth) on the first hit,
	// which is proportional to cos(theta)/r - 1/far instead of 1/r. The leverage of a ray
	// then depends on its direction, so the rest pose is not reproduced.
	PMVCMirrorParams legacy = singleHit;
	legacy.legacyDepthWeight = true;
	const Eigen::MatrixXd legacyWeights = computePMVCMirror(uCage, samples, legacy);
	const double legacyResidual = linearReproductionResidual(legacyWeights, uCage, samples);
	std::cout << "linear reproduction residual of the legacy (1 - depth) weighting: "
		<< legacyResidual << std::endl;
	check(legacyResidual > 1e-4,
		"the legacy (1 - depth) weighting does not reproduce the rest pose");

	// Skipping the entry hits still goes through the same per-ray normalization, so it
	// keeps both properties; it only changes which hits receive the weight.
	PMVCMirrorParams skipEven = fiveHits;
	skipEven.skipEvenHits = true;
	const Eigen::MatrixXd skipped = computePMVCMirror(uCage, samples, skipEven);
	check(skipped.allFinite() && isNormalized(skipped), "skip-even-hits weights are finite and normalized");
	check((skipped.array() >= 0.0).all(), "skip-even-hits weights are non-negative");
	check(linearReproductionResidual(skipped, uCage, samples) < 1e-12,
		"skip-even-hits weights still reproduce the rest pose exactly");
	check((skipped - five).cwiseAbs().maxCoeff() > 0.0, "skipping the entry hits changes the weights");

	// Interior distances bias the split along a ray. They cannot act on the first hit,
	// where the straight segment lies inside the cage and the detour is zero by
	// construction, so they need more than one hit to change anything.
	PMVCMirrorParams interiorSingle = singleHit;
	interiorSingle.interiorDetours = &detours;
	const Eigen::MatrixXd interiorSingleWeights = computePMVCMirror(uCage, samples, interiorSingle);
	check((interiorSingleWeights - single).cwiseAbs().maxCoeff() == 0.0,
		"interior distances cannot change the single-hit variant, the first hit has no detour");

	PMVCMirrorParams interiorMulti = fiveHits;
	interiorMulti.interiorDetours = &detours;
	const Eigen::MatrixXd interiorMultiWeights = computePMVCMirror(uCage, samples, interiorMulti);
	check(interiorMultiWeights.allFinite(), "interior-distance weights are finite");
	check(isNormalized(interiorMultiWeights), "interior-distance weights are normalized");
	check((interiorMultiWeights.array() >= 0.0).all(), "interior-distance weights are non-negative");
	check((interiorMultiWeights - five).cwiseAbs().maxCoeff() > 0.0,
		"interior distances change the multi-hit weights on the non-convex cage");
	check(linearReproductionResidual(interiorMultiWeights, uCage, samples) < 1e-12,
		"interior-distance weights still reproduce the rest pose exactly");

	// Zero detours must reduce the variant exactly to the Euclidean one.
	PMVCMirrorParams zeroDetourMulti = fiveHits;
	zeroDetourMulti.interiorDetours = &zeroDetours;
	const Eigen::MatrixXd zeroDetourWeights = computePMVCMirror(uCage, samples, zeroDetourMulti);
	check((zeroDetourWeights - five).cwiseAbs().maxCoeff() < 1e-12,
		"zero detours reduce the interior variant exactly to the Euclidean multi-hit weights");
}

/**
 * The invariants must not depend on the peeling depth. Every ray is normalized over the
 * hits it actually has, so truncating a ray that crosses the cage more often than the
 * rendered layers allow changes which weights it gets but not that they sum to one.
 */
void testPhase8HitCountSweep()
{
	std::cout << "\n=== Phase 8: invariants across the peeling depth ===" << std::endl;

	const TestMesh uCage = makeUCage(1.0);
	Eigen::MatrixXd samples(3, 3);
	samples << 0.5, 2.0, 0.5,   // left arm, rays towards +x cross into the right arm
		1.5, 0.5, 0.5,          // base
		2.5, 2.0, 0.5;          // right arm

	InteriorDistanceParams distanceParams;
	distanceParams.resolution = 48;
	Eigen::MatrixXf table;
	computeInteriorDistances(uCage.V, uCage.F, samples, distanceParams, table);
	const Eigen::MatrixXf detours = (table - euclideanDistanceTable(uCage, samples)).cwiseMax(0.f);

	bool allExact = true;
	bool allPositive = true;
	bool allNormalized = true;

	for (uint32_t hitCount = 1; hitCount <= 8; ++hitCount)
	{
		for (int variant = 0; variant < 3; ++variant)
		{
			PMVCMirrorParams params;
			params.faceSize = 16;
			params.hitCount = hitCount;
			params.skipEvenHits = (variant == 1);
			params.interiorDetours = (variant == 2) ? &detours : nullptr;

			const Eigen::MatrixXd weights = computePMVCMirror(uCage, samples, params);
			const double residual = linearReproductionResidual(weights, uCage, samples);

			double rowSumError = 0.0;
			for (Eigen::Index row = 0; row < weights.rows(); ++row)
			{
				rowSumError = std::max(rowSumError, std::abs(weights.row(row).sum() - 1.0));
			}

			allExact = allExact && residual < 1e-12;
			allPositive = allPositive && (weights.array() >= 0.0).all() && weights.allFinite();
			allNormalized = allNormalized && rowSumError < 1e-9;

			if (variant == 0)
			{
				std::cout << "  hitCount=" << hitCount << " residual=" << residual
					<< " minWeight=" << weights.minCoeff() << std::endl;
			}
		}
	}

	check(allExact, "linear reproduction is exact for every peeling depth and variant");
	check(allPositive, "weights stay finite and non-negative for every peeling depth and variant");
	check(allNormalized, "weights stay normalized for every peeling depth and variant");
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
	testPhase8HitCountSweep();

	std::cout << "\n" << (failures == 0 ? "ALL CHECKS PASSED" : "CHECKS FAILED") << " (" << failures << " failures)" << std::endl;
	return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
