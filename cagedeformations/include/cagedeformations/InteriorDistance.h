#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

/**
 * Interior (volumetric) distances between cage vertices and mesh vertices, computed with
 * the Heat Method (Crane, Weischedel, Wardetzky, "Geodesics in Heat", TOG 2013) on a
 * voxelization of the cage interior. Unlike the Euclidean distance used by the classic
 * PMVC weighting, these distances respect the cage geometry: they never take shortcuts
 * through cage walls, so two points separated by a thin wall of a non-convex cage are
 * far apart even when they are close in Euclidean space.
 */

enum VoxelLabel : uint8_t
{
	VOXEL_EXTERIOR = 0,
	VOXEL_INTERIOR = 1,
	VOXEL_BOUNDARY = 2
};

struct VoxelGrid
{
	int Nx = 0;
	int Ny = 0;
	int Nz = 0;
	float voxelSize = 0.f;
	/// World-space position of the minimum corner of voxel (0, 0, 0).
	Eigen::Vector3f origin = Eigen::Vector3f::Zero();
	/// One VoxelLabel per voxel, x-fastest layout: labels[x + Nx * (y + Ny * z)].
	std::vector<uint8_t> labels;

	size_t index(int x, int y, int z) const
	{
		return static_cast<size_t>(x) + static_cast<size_t>(Nx) * (static_cast<size_t>(y) + static_cast<size_t>(Ny) * static_cast<size_t>(z));
	}

	bool inBounds(int x, int y, int z) const
	{
		return x >= 0 && y >= 0 && z >= 0 && x < Nx && y < Ny && z < Nz;
	}

	uint8_t label(int x, int y, int z) const
	{
		return labels[index(x, y, z)];
	}

	/// A voxel is solid when it belongs to the simulation domain (cage interior or its boundary shell).
	bool isSolid(int x, int y, int z) const
	{
		return label(x, y, z) != VOXEL_EXTERIOR;
	}

	Eigen::Vector3f voxelCenter(int x, int y, int z) const
	{
		return origin + voxelSize * Eigen::Vector3f(static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f, static_cast<float>(z) + 0.5f);
	}
};

struct InteriorDistanceParams
{
	/// Number of voxels along the longest axis of the cage bounding box (the other axes
	/// scale proportionally). The grid always keeps a 2 voxel exterior margin so the
	/// outside is connected for the flood fill.
	int resolution = 64;
	/// Symmetric Gauss-Seidel sweeps for the heat step (I - t * Laplacian) u = delta.
	int heatIterations = 100;
	/// Symmetric Gauss-Seidel sweeps for the Poisson step Laplacian(phi) = div(X). The
	/// solve starts from the interior graph distance, so only local refinement is needed.
	int poissonIterations = 200;
	/// Heat time step multiplier m in t = m * h^2 (h = voxel size).
	double timeStepScale = 1.0;
};

/**
 * Voxelizes the interior volume enclosed by a watertight triangle cage.
 * Voxels overlapping a cage triangle become VOXEL_BOUNDARY, voxels enclosed by the cage
 * become VOXEL_INTERIOR and everything reachable from the grid border without crossing
 * the boundary shell becomes VOXEL_EXTERIOR (flood fill, so the classification cannot
 * leak through walls that are at least one voxel thick at the chosen resolution).
 */
VoxelGrid voxelizeCageInterior(const Eigen::MatrixXd& cageVertices, const Eigen::MatrixXi& cageFaces, int resolution);

/**
 * Heat-method distance field over the solid voxels of the grid from a single source point:
 *   1. solve (I - t * Laplacian) u = delta  (heat diffusion from the source voxel)
 *   2. X = -grad(u) / |grad(u)|
 *   3. solve Laplacian(phi) = div(X)        (Poisson, recovers the distance)
 * Exterior voxels are excluded from every stencil, which realizes the zero Neumann
 * boundary condition at the cage walls. The result is stored per voxel (exterior voxels
 * receive -1) and is shifted so the source voxel has distance 0.
 */
void computeHeatDistanceField(const VoxelGrid& grid, const Eigen::Vector3d& sourcePoint,
	const InteriorDistanceParams& params, std::vector<float>& outDistances);

/**
 * Full interior distance table for a cage/mesh pair.
 * Row layout matches interiorDistance[cageVertexIdx][meshVertexIdx]: entry (c, m) is the
 * interior distance between cage vertex c and mesh vertex m, obtained by solving one
 * heat-method distance field per cage vertex (source snapped to the nearest solid voxel)
 * and sampling it at every mesh vertex with trilinear interpolation over the 8
 * surrounding voxel centers (exterior corners are dropped and the weights renormalized).
 * Pairs the solver cannot reach (empty grid, degenerate cage) fall back to Euclidean
 * distance so the table never contains NaNs or negative values.
 */
void computeInteriorDistances(const Eigen::MatrixXd& cageVertices, const Eigen::MatrixXi& cageFaces,
	const Eigen::MatrixXd& meshVertices, const InteriorDistanceParams& params, Eigen::MatrixXf& outDistances);

/**
 * Writes the solid voxels of the grid as unit cubes to an OBJ file for visual inspection
 * (interior and boundary voxels get separate object groups).
 */
bool writeVoxelGridOBJ(const std::string& fileName, const VoxelGrid& grid);
