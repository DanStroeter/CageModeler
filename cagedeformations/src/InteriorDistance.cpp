#include <cagedeformations/InteriorDistance.h>
#include <cagedeformations/globals.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <thread>
#include <utility>

namespace
{

// --------------------------------------------------------------------------------------
// Triangle / axis-aligned box overlap (Akenine-Moller separating axis test).
// --------------------------------------------------------------------------------------

bool axisTest(const Eigen::Vector3f& axis, const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
	const Eigen::Vector3f& v2, const Eigen::Vector3f& halfSize)
{
	const float p0 = axis.dot(v0);
	const float p1 = axis.dot(v1);
	const float p2 = axis.dot(v2);
	const float r = halfSize.x() * std::abs(axis.x()) + halfSize.y() * std::abs(axis.y()) + halfSize.z() * std::abs(axis.z());
	const float minP = std::min({ p0, p1, p2 });
	const float maxP = std::max({ p0, p1, p2 });

	return !(minP > r || maxP < -r);
}

bool triangleIntersectsBox(const Eigen::Vector3f& boxCenter, const Eigen::Vector3f& halfSize,
	const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c)
{
	const Eigen::Vector3f v0 = a - boxCenter;
	const Eigen::Vector3f v1 = b - boxCenter;
	const Eigen::Vector3f v2 = c - boxCenter;

	// Box face normals.
	for (int axis = 0; axis < 3; ++axis)
	{
		const float minV = std::min({ v0[axis], v1[axis], v2[axis] });
		const float maxV = std::max({ v0[axis], v1[axis], v2[axis] });
		if (minV > halfSize[axis] || maxV < -halfSize[axis])
		{
			return false;
		}
	}

	const Eigen::Vector3f e0 = v1 - v0;
	const Eigen::Vector3f e1 = v2 - v1;
	const Eigen::Vector3f e2 = v0 - v2;

	// Triangle normal.
	if (!axisTest(e0.cross(e1), v0, v1, v2, halfSize))
	{
		return false;
	}

	// Nine cross products of box axes and triangle edges.
	const Eigen::Vector3f boxAxes[3] = { Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), Eigen::Vector3f::UnitZ() };
	const Eigen::Vector3f edges[3] = { e0, e1, e2 };
	for (const auto& boxAxis : boxAxes)
	{
		for (const auto& edge : edges)
		{
			const Eigen::Vector3f axis = boxAxis.cross(edge);
			if (axis.squaredNorm() > 1e-12f && !axisTest(axis, v0, v1, v2, halfSize))
			{
				return false;
			}
		}
	}

	return true;
}

// --------------------------------------------------------------------------------------
// Compact representation of the solid (interior + boundary) voxels for the solver.
// --------------------------------------------------------------------------------------

struct SolidDomain
{
	/// Maps a voxel index to its slot in the compact arrays, -1 for exterior voxels.
	std::vector<int> solidId;
	/// Voxel coordinate per solid slot, in lexicographic (z, y, x) order.
	std::vector<Eigen::Vector3i> coords;
	/// The 6-neighborhood per solid slot (-x, +x, -y, +y, -z, +z), -1 when the neighbor is exterior.
	std::vector<std::array<int, 6>> neighbors;
	std::vector<uint8_t> degree;
};

SolidDomain buildSolidDomain(const VoxelGrid& grid)
{
	SolidDomain domain;
	domain.solidId.assign(grid.labels.size(), -1);

	int count = 0;
	for (int z = 0; z < grid.Nz; ++z)
	{
		for (int y = 0; y < grid.Ny; ++y)
		{
			for (int x = 0; x < grid.Nx; ++x)
			{
				if (grid.isSolid(x, y, z))
				{
					domain.solidId[grid.index(x, y, z)] = count++;
					domain.coords.emplace_back(x, y, z);
				}
			}
		}
	}

	static constexpr int offsets[6][3] = { { -1, 0, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { 0, 1, 0 }, { 0, 0, -1 }, { 0, 0, 1 } };

	domain.neighbors.resize(count);
	domain.degree.resize(count);
	for (int i = 0; i < count; ++i)
	{
		const Eigen::Vector3i& v = domain.coords[i];
		uint8_t degree = 0;
		for (int n = 0; n < 6; ++n)
		{
			const int nx = v.x() + offsets[n][0];
			const int ny = v.y() + offsets[n][1];
			const int nz = v.z() + offsets[n][2];
			const int id = grid.inBounds(nx, ny, nz) ? domain.solidId[grid.index(nx, ny, nz)] : -1;
			domain.neighbors[i][n] = id;
			if (id >= 0)
			{
				++degree;
			}
		}
		domain.degree[i] = degree;
	}

	return domain;
}

int findNearestSolidVoxel(const VoxelGrid& grid, const SolidDomain& domain, const Eigen::Vector3d& point)
{
	if (domain.coords.empty())
	{
		return -1;
	}

	const Eigen::Vector3f p = point.cast<float>();
	const Eigen::Vector3f local = (p - grid.origin) / grid.voxelSize;
	const int cx = std::min(std::max(static_cast<int>(std::floor(local.x())), 0), grid.Nx - 1);
	const int cy = std::min(std::max(static_cast<int>(std::floor(local.y())), 0), grid.Ny - 1);
	const int cz = std::min(std::max(static_cast<int>(std::floor(local.z())), 0), grid.Nz - 1);

	const int maxRadius = std::max({ grid.Nx, grid.Ny, grid.Nz });
	for (int radius = 0; radius <= maxRadius; ++radius)
	{
		int best = -1;
		float bestDistSq = std::numeric_limits<float>::max();
		for (int z = std::max(0, cz - radius); z <= std::min(grid.Nz - 1, cz + radius); ++z)
		{
			for (int y = std::max(0, cy - radius); y <= std::min(grid.Ny - 1, cy + radius); ++y)
			{
				for (int x = std::max(0, cx - radius); x <= std::min(grid.Nx - 1, cx + radius); ++x)
				{
					const int id = domain.solidId[grid.index(x, y, z)];
					if (id < 0)
					{
						continue;
					}
					const float distSq = (grid.voxelCenter(x, y, z) - p).squaredNorm();
					if (distSq < bestDistSq)
					{
						bestDistSq = distSq;
						best = id;
					}
				}
			}
		}
		if (best >= 0)
		{
			return best;
		}
	}

	return -1;
}

/**
 * Symmetric Gauss-Seidel sweeps for the screened system u_i * (1 + a * deg_i) - a * sum(u_j) = rhs_i.
 * With a > 0 this solves the heat step (I - t * Laplacian) u = rhs where a = t / h^2; exterior
 * neighbors are simply absent from the stencil which realizes the zero Neumann condition.
 */
void gaussSeidelScreened(const SolidDomain& domain, const std::vector<double>& rhs, double a,
	int iterations, std::vector<double>& u)
{
	const int count = static_cast<int>(domain.coords.size());
	for (int iteration = 0; iteration < iterations; ++iteration)
	{
		for (int i = 0; i < count; ++i)
		{
			double sum = 0.0;
			for (const int j : domain.neighbors[i])
			{
				if (j >= 0)
				{
					sum += u[j];
				}
			}
			u[i] = (rhs[i] + a * sum) / (1.0 + a * static_cast<double>(domain.degree[i]));
		}
		for (int i = count - 1; i >= 0; --i)
		{
			double sum = 0.0;
			for (const int j : domain.neighbors[i])
			{
				if (j >= 0)
				{
					sum += u[j];
				}
			}
			u[i] = (rhs[i] + a * sum) / (1.0 + a * static_cast<double>(domain.degree[i]));
		}
	}
}

/**
 * Symmetric Gauss-Seidel sweeps for the pure Neumann Poisson system Laplacian(phi) = rhs,
 * discretized as (sum(phi_j) - deg_i * phi_i) / h^2 = rhs_i. The system is only defined up
 * to a constant which the caller removes by shifting.
 */
void gaussSeidelPoisson(const SolidDomain& domain, const std::vector<double>& rhs, double hSq,
	int iterations, std::vector<double>& phi)
{
	const int count = static_cast<int>(domain.coords.size());
	for (int iteration = 0; iteration < iterations; ++iteration)
	{
		for (int i = 0; i < count; ++i)
		{
			if (domain.degree[i] == 0)
			{
				continue;
			}
			double sum = 0.0;
			for (const int j : domain.neighbors[i])
			{
				if (j >= 0)
				{
					sum += phi[j];
				}
			}
			phi[i] = (sum - hSq * rhs[i]) / static_cast<double>(domain.degree[i]);
		}
		for (int i = count - 1; i >= 0; --i)
		{
			if (domain.degree[i] == 0)
			{
				continue;
			}
			double sum = 0.0;
			for (const int j : domain.neighbors[i])
			{
				if (j >= 0)
				{
					sum += phi[j];
				}
			}
			phi[i] = (sum - hSq * rhs[i]) / static_cast<double>(domain.degree[i]);
		}
	}
}

/**
 * Normalized negative heat gradient X = -grad(u) / |grad(u)| per solid voxel, using central
 * differences where both axis neighbors exist and one-sided differences at the walls.
 */
void computeNormalizedNegativeGradient(const SolidDomain& domain, const std::vector<double>& u,
	double h, std::vector<Eigen::Vector3d>& X)
{
	const int count = static_cast<int>(domain.coords.size());
	X.assign(count, Eigen::Vector3d::Zero());

	for (int i = 0; i < count; ++i)
	{
		Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
		for (int axis = 0; axis < 3; ++axis)
		{
			const int negNeighbor = domain.neighbors[i][axis * 2];
			const int posNeighbor = domain.neighbors[i][axis * 2 + 1];
			if (negNeighbor >= 0 && posNeighbor >= 0)
			{
				gradient[axis] = (u[posNeighbor] - u[negNeighbor]) / (2.0 * h);
			}
			else if (posNeighbor >= 0)
			{
				gradient[axis] = (u[posNeighbor] - u[i]) / h;
			}
			else if (negNeighbor >= 0)
			{
				gradient[axis] = (u[i] - u[negNeighbor]) / h;
			}
		}

		const double norm = gradient.norm();
		if (norm > 1e-300)
		{
			X[i] = -gradient / norm;
		}
	}
}

/**
 * Finite volume divergence of X: fluxes through the 6 voxel faces, zero flux through faces
 * shared with exterior voxels (Neumann walls).
 */
void computeDivergence(const SolidDomain& domain, const std::vector<Eigen::Vector3d>& X,
	double h, std::vector<double>& divergence)
{
	const int count = static_cast<int>(domain.coords.size());
	divergence.assign(count, 0.0);

	for (int i = 0; i < count; ++i)
	{
		double div = 0.0;
		for (int axis = 0; axis < 3; ++axis)
		{
			const int negNeighbor = domain.neighbors[i][axis * 2];
			const int posNeighbor = domain.neighbors[i][axis * 2 + 1];
			if (posNeighbor >= 0)
			{
				div += 0.5 * (X[i][axis] + X[posNeighbor][axis]);
			}
			if (negNeighbor >= 0)
			{
				div -= 0.5 * (X[i][axis] + X[negNeighbor][axis]);
			}
		}
		divergence[i] = div / h;
	}
}

/**
 * Graph distance from the source voxel over the 26-neighborhood of the solid voxels
 * (Dijkstra with Euclidean edge lengths). This respects the interior topology exactly and
 * overestimates the true geodesic by only a few percent, which makes it an excellent
 * initial guess for the Poisson stage: the Gauss-Seidel sweeps then only have to smooth
 * out the local grid anisotropy instead of transporting distance information across the
 * whole domain.
 */
void dijkstraGraphDistance(const VoxelGrid& grid, const SolidDomain& domain, int sourceSolidId,
	std::vector<double>& distances)
{
	const int count = static_cast<int>(domain.coords.size());
	distances.assign(count, std::numeric_limits<double>::max());
	distances[sourceSolidId] = 0.0;

	using QueueEntry = std::pair<double, int>;
	std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
	queue.emplace(0.0, sourceSolidId);

	const double h = static_cast<double>(grid.voxelSize);
	while (!queue.empty())
	{
		const QueueEntry entry = queue.top();
		const double distance = entry.first;
		const int i = entry.second;
		queue.pop();
		if (distance > distances[i])
		{
			continue;
		}

		const Eigen::Vector3i& v = domain.coords[i];
		for (int dz = -1; dz <= 1; ++dz)
		{
			for (int dy = -1; dy <= 1; ++dy)
			{
				for (int dx = -1; dx <= 1; ++dx)
				{
					if (dx == 0 && dy == 0 && dz == 0)
					{
						continue;
					}
					const int nx = v.x() + dx, ny = v.y() + dy, nz = v.z() + dz;
					if (!grid.inBounds(nx, ny, nz))
					{
						continue;
					}
					const int j = domain.solidId[grid.index(nx, ny, nz)];
					if (j < 0)
					{
						continue;
					}
					const double candidate = distance + h * std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
					if (candidate < distances[j])
					{
						distances[j] = candidate;
						queue.emplace(candidate, j);
					}
				}
			}
		}
	}

	// Voxels disconnected from the source (should not happen for a watertight cage) fall
	// back to the Euclidean distance so downstream consumers never see infinities.
	const Eigen::Vector3i& src = domain.coords[sourceSolidId];
	const Eigen::Vector3f sourceCenter = grid.voxelCenter(src.x(), src.y(), src.z());
	for (int i = 0; i < count; ++i)
	{
		if (distances[i] == std::numeric_limits<double>::max())
		{
			const Eigen::Vector3i& v = domain.coords[i];
			distances[i] = static_cast<double>((grid.voxelCenter(v.x(), v.y(), v.z()) - sourceCenter).norm());
		}
	}
}

void solveDistanceField(const VoxelGrid& grid, const SolidDomain& domain, int sourceSolidId,
	const InteriorDistanceParams& params, std::vector<double>& phi)
{
	const int count = static_cast<int>(domain.coords.size());
	const double h = static_cast<double>(grid.voxelSize);

	// Stage 1: heat diffusion from the source voxel, (I - t * Laplacian) u = delta.
	std::vector<double> rhs(count, 0.0);
	rhs[sourceSolidId] = 1.0;
	std::vector<double> u(count, 0.0);
	gaussSeidelScreened(domain, rhs, params.timeStepScale, params.heatIterations, u);

	// Stage 2: normalized negative gradient of the heat.
	std::vector<Eigen::Vector3d> X;
	computeNormalizedNegativeGradient(domain, u, h, X);

	// Stage 3: Poisson solve Laplacian(phi) = div(X), initialized with the interior graph
	// distance so the iterative solve only needs local refinement.
	std::vector<double> divergence;
	computeDivergence(domain, X, h, divergence);

	dijkstraGraphDistance(grid, domain, sourceSolidId, phi);

	gaussSeidelPoisson(domain, divergence, h * h, params.poissonIterations, phi);

	// The Neumann Poisson solution is defined up to a constant: pin the source to zero.
	const double sourceValue = phi[sourceSolidId];
	for (double& value : phi)
	{
		value = std::max(0.0, value - sourceValue);
	}
}

/**
 * Trilinear interpolation of a per-voxel field at a world-space point. Exterior corners are
 * excluded and the remaining weights renormalized; when no surrounding voxel carries a value
 * the nearest solid voxel is used instead. Returns a negative value only when the grid has
 * no solid voxels at all.
 */
double sampleFieldTrilinear(const VoxelGrid& grid, const SolidDomain& domain,
	const std::vector<double>& field, const Eigen::Vector3d& point)
{
	const Eigen::Vector3f local = (point.cast<float>() - grid.origin) / grid.voxelSize - Eigen::Vector3f::Constant(0.5f);
	const int baseX = static_cast<int>(std::floor(local.x()));
	const int baseY = static_cast<int>(std::floor(local.y()));
	const int baseZ = static_cast<int>(std::floor(local.z()));
	const float fx = local.x() - static_cast<float>(baseX);
	const float fy = local.y() - static_cast<float>(baseY);
	const float fz = local.z() - static_cast<float>(baseZ);

	double weightedSum = 0.0;
	double weightSum = 0.0;
	for (int dz = 0; dz < 2; ++dz)
	{
		for (int dy = 0; dy < 2; ++dy)
		{
			for (int dx = 0; dx < 2; ++dx)
			{
				const int x = baseX + dx;
				const int y = baseY + dy;
				const int z = baseZ + dz;
				if (!grid.inBounds(x, y, z))
				{
					continue;
				}
				const int id = domain.solidId[grid.index(x, y, z)];
				if (id < 0)
				{
					continue;
				}
				const double w = static_cast<double>((dx != 0 ? fx : 1.f - fx) * (dy != 0 ? fy : 1.f - fy) * (dz != 0 ? fz : 1.f - fz));
				weightedSum += w * field[id];
				weightSum += w;
			}
		}
	}

	if (weightSum > 1e-9)
	{
		return weightedSum / weightSum;
	}

	const int nearest = findNearestSolidVoxel(grid, domain, point);
	if (nearest >= 0)
	{
		return field[nearest];
	}

	return -1.0;
}

} // namespace

VoxelGrid voxelizeCageInterior(const Eigen::MatrixXd& cageVertices, const Eigen::MatrixXi& cageFaces, int resolution)
{
	VoxelGrid grid;
	if (cageVertices.rows() == 0 || cageVertices.cols() < 3 || cageFaces.rows() == 0 || resolution < 4)
	{
		return grid;
	}

	const Eigen::Vector3f bboxMin = cageVertices.leftCols<3>().colwise().minCoeff().cast<float>();
	const Eigen::Vector3f bboxMax = cageVertices.leftCols<3>().colwise().maxCoeff().cast<float>();
	const Eigen::Vector3f extent = bboxMax - bboxMin;
	const float maxExtent = extent.maxCoeff();
	if (maxExtent <= 0.f)
	{
		return grid;
	}

	constexpr int margin = 2;
	const float h = maxExtent / static_cast<float>(resolution);
	grid.voxelSize = h;
	grid.origin = bboxMin - Eigen::Vector3f::Constant(static_cast<float>(margin) * h);
	grid.Nx = static_cast<int>(std::ceil(extent.x() / h)) + 2 * margin;
	grid.Ny = static_cast<int>(std::ceil(extent.y() / h)) + 2 * margin;
	grid.Nz = static_cast<int>(std::ceil(extent.z() / h)) + 2 * margin;
	grid.labels.assign(static_cast<size_t>(grid.Nx) * grid.Ny * grid.Nz, VOXEL_INTERIOR);

	// Conservative boundary shell: every voxel whose box overlaps a cage triangle. The box
	// is inflated by a small epsilon so triangles lying exactly on voxel boundary planes
	// (common for axis-aligned cages) cannot slip between two voxel layers through float
	// rounding.
	const Eigen::Vector3f halfSize = Eigen::Vector3f::Constant(0.5f * h * (1.f + 1e-3f));
	for (int face = 0; face < cageFaces.rows(); ++face)
	{
		const int i0 = cageFaces(face, 0);
		const int i1 = cageFaces(face, 1);
		const int i2 = cageFaces(face, 2);
		if (i0 < 0 || i1 < 0 || i2 < 0 || i0 >= cageVertices.rows() || i1 >= cageVertices.rows() || i2 >= cageVertices.rows())
		{
			continue;
		}

		const Eigen::Vector3f a = cageVertices.row(i0).leftCols<3>().cast<float>();
		const Eigen::Vector3f b = cageVertices.row(i1).leftCols<3>().cast<float>();
		const Eigen::Vector3f c = cageVertices.row(i2).leftCols<3>().cast<float>();

		const Eigen::Vector3f triMin = a.cwiseMin(b).cwiseMin(c);
		const Eigen::Vector3f triMax = a.cwiseMax(b).cwiseMax(c);
		const Eigen::Vector3f localMin = (triMin - grid.origin) / h;
		const Eigen::Vector3f localMax = (triMax - grid.origin) / h;

		const int minX = std::min(std::max(static_cast<int>(std::floor(localMin.x())), 0), grid.Nx - 1);
		const int minY = std::min(std::max(static_cast<int>(std::floor(localMin.y())), 0), grid.Ny - 1);
		const int minZ = std::min(std::max(static_cast<int>(std::floor(localMin.z())), 0), grid.Nz - 1);
		const int maxX = std::min(std::max(static_cast<int>(std::floor(localMax.x())), 0), grid.Nx - 1);
		const int maxY = std::min(std::max(static_cast<int>(std::floor(localMax.y())), 0), grid.Ny - 1);
		const int maxZ = std::min(std::max(static_cast<int>(std::floor(localMax.z())), 0), grid.Nz - 1);

		for (int z = minZ; z <= maxZ; ++z)
		{
			for (int y = minY; y <= maxY; ++y)
			{
				for (int x = minX; x <= maxX; ++x)
				{
					const size_t idx = grid.index(x, y, z);
					if (grid.labels[idx] == VOXEL_BOUNDARY)
					{
						continue;
					}
					if (triangleIntersectsBox(grid.voxelCenter(x, y, z), halfSize, a, b, c))
					{
						grid.labels[idx] = VOXEL_BOUNDARY;
					}
				}
			}
		}
	}

	// Flood fill the outside from the grid border: everything reachable without crossing
	// the boundary shell is exterior, whatever remains enclosed is the cage interior.
	std::deque<Eigen::Vector3i> queue;
	auto pushExterior = [&grid, &queue](int x, int y, int z)
	{
		const size_t idx = grid.index(x, y, z);
		if (grid.labels[idx] == VOXEL_INTERIOR)
		{
			grid.labels[idx] = VOXEL_EXTERIOR;
			queue.emplace_back(x, y, z);
		}
	};

	for (int z = 0; z < grid.Nz; ++z)
	{
		for (int y = 0; y < grid.Ny; ++y)
		{
			for (int x = 0; x < grid.Nx; ++x)
			{
				if (x == 0 || y == 0 || z == 0 || x == grid.Nx - 1 || y == grid.Ny - 1 || z == grid.Nz - 1)
				{
					pushExterior(x, y, z);
				}
			}
		}
	}

	static constexpr int offsets[6][3] = { { -1, 0, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { 0, 1, 0 }, { 0, 0, -1 }, { 0, 0, 1 } };
	while (!queue.empty())
	{
		const Eigen::Vector3i v = queue.front();
		queue.pop_front();
		for (const auto& offset : offsets)
		{
			const int x = v.x() + offset[0];
			const int y = v.y() + offset[1];
			const int z = v.z() + offset[2];
			if (grid.inBounds(x, y, z))
			{
				pushExterior(x, y, z);
			}
		}
	}

	return grid;
}

void computeHeatDistanceField(const VoxelGrid& grid, const Eigen::Vector3d& sourcePoint,
	const InteriorDistanceParams& params, std::vector<float>& outDistances)
{
	outDistances.assign(grid.labels.size(), -1.f);

	const SolidDomain domain = buildSolidDomain(grid);
	const int sourceSolidId = findNearestSolidVoxel(grid, domain, sourcePoint);
	if (sourceSolidId < 0)
	{
		return;
	}

	std::vector<double> phi;
	solveDistanceField(grid, domain, sourceSolidId, params, phi);

	for (size_t i = 0; i < domain.coords.size(); ++i)
	{
		const Eigen::Vector3i& v = domain.coords[i];
		outDistances[grid.index(v.x(), v.y(), v.z())] = static_cast<float>(phi[i]);
	}
}

void computeInteriorDistances(const Eigen::MatrixXd& cageVertices, const Eigen::MatrixXi& cageFaces,
	const Eigen::MatrixXd& meshVertices, const InteriorDistanceParams& params, Eigen::MatrixXf& outDistances)
{
	const Eigen::Index numCageVertices = cageVertices.rows();
	const Eigen::Index numMeshVertices = meshVertices.rows();
	outDistances.resize(numCageVertices, numMeshVertices);

	auto fillEuclideanRow = [&](Eigen::Index cage)
	{
		const Eigen::Vector3d cagePosition = cageVertices.row(cage).leftCols<3>();
		for (Eigen::Index mesh = 0; mesh < numMeshVertices; ++mesh)
		{
			const Eigen::Vector3d meshPosition = meshVertices.row(mesh).leftCols<3>();
			outDistances(cage, mesh) = static_cast<float>((meshPosition - cagePosition).norm());
		}
	};

	const VoxelGrid grid = voxelizeCageInterior(cageVertices, cageFaces, params.resolution);
	const SolidDomain domain = grid.labels.empty() ? SolidDomain{} : buildSolidDomain(grid);
	if (domain.coords.empty())
	{
		if (verbosity > 0)
		{
			std::cout << "computeInteriorDistances: voxelization produced no solid voxels, falling back to Euclidean distances" << std::endl;
		}
		for (Eigen::Index cage = 0; cage < numCageVertices; ++cage)
		{
			fillEuclideanRow(cage);
		}
		return;
	}

	if (verbosity > 0)
	{
		std::cout << "computeInteriorDistances: grid " << grid.Nx << "x" << grid.Ny << "x" << grid.Nz
			<< " (" << domain.coords.size() << " solid voxels), " << numCageVertices << " heat solves" << std::endl;
	}

	// One heat-method solve per cage vertex; the solves are independent, spread them over threads.
	const unsigned int hardwareThreads = std::max(1u, std::thread::hardware_concurrency());
	const unsigned int workerCount = static_cast<unsigned int>(std::min<Eigen::Index>(hardwareThreads, numCageVertices));

	auto solveRange = [&](Eigen::Index begin, Eigen::Index end)
	{
		for (Eigen::Index cage = begin; cage < end; ++cage)
		{
			const Eigen::Vector3d cagePosition = cageVertices.row(cage).leftCols<3>();
			const int sourceSolidId = findNearestSolidVoxel(grid, domain, cagePosition);
			if (sourceSolidId < 0)
			{
				fillEuclideanRow(cage);
				continue;
			}

			std::vector<double> phi;
			solveDistanceField(grid, domain, sourceSolidId, params, phi);

			for (Eigen::Index mesh = 0; mesh < numMeshVertices; ++mesh)
			{
				const Eigen::Vector3d meshPosition = meshVertices.row(mesh).leftCols<3>();
				const double distance = sampleFieldTrilinear(grid, domain, phi, meshPosition);
				outDistances(cage, mesh) = distance >= 0.0
					? static_cast<float>(distance)
					: static_cast<float>((meshPosition - cagePosition).norm());
			}
		}
	};

	if (workerCount <= 1)
	{
		solveRange(0, numCageVertices);
	}
	else
	{
		std::vector<std::thread> workers;
		workers.reserve(workerCount);
		const Eigen::Index chunk = (numCageVertices + workerCount - 1) / workerCount;
		for (unsigned int worker = 0; worker < workerCount; ++worker)
		{
			const Eigen::Index begin = static_cast<Eigen::Index>(worker) * chunk;
			const Eigen::Index end = std::min(numCageVertices, begin + chunk);
			if (begin >= end)
			{
				break;
			}
			workers.emplace_back(solveRange, begin, end);
		}
		for (auto& worker : workers)
		{
			worker.join();
		}
	}
}

bool writeVoxelGridOBJ(const std::string& fileName, const VoxelGrid& grid)
{
	std::ofstream file(fileName);
	if (!file.is_open())
	{
		return false;
	}

	int vertexOffset = 1;
	auto writeCube = [&file, &vertexOffset, &grid](int x, int y, int z)
	{
		const Eigen::Vector3f minCorner = grid.origin + grid.voxelSize * Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
		for (int corner = 0; corner < 8; ++corner)
		{
			const Eigen::Vector3f p = minCorner + grid.voxelSize * Eigen::Vector3f(
				static_cast<float>(corner & 1), static_cast<float>((corner >> 1) & 1), static_cast<float>((corner >> 2) & 1));
			file << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
		}
		static constexpr int quads[6][4] = {
			{ 0, 2, 3, 1 }, { 4, 5, 7, 6 }, { 0, 1, 5, 4 }, { 2, 6, 7, 3 }, { 0, 4, 6, 2 }, { 1, 3, 7, 5 }
		};
		for (const auto& quad : quads)
		{
			file << "f " << vertexOffset + quad[0] << " " << vertexOffset + quad[1] << " "
				<< vertexOffset + quad[2] << " " << vertexOffset + quad[3] << "\n";
		}
		vertexOffset += 8;
	};

	for (const uint8_t targetLabel : { static_cast<uint8_t>(VOXEL_INTERIOR), static_cast<uint8_t>(VOXEL_BOUNDARY) })
	{
		file << (targetLabel == VOXEL_INTERIOR ? "o interior_voxels\n" : "o boundary_voxels\n");
		for (int z = 0; z < grid.Nz; ++z)
		{
			for (int y = 0; y < grid.Ny; ++y)
			{
				for (int x = 0; x < grid.Nx; ++x)
				{
					if (grid.label(x, y, z) == targetLabel)
					{
						writeCube(x, y, z);
					}
				}
			}
		}
	}

	return true;
}
