#include <cagedeformations/LoadMesh.h>
#include <cagedeformations/globals.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <unordered_map>

#include <igl/readMSH.h>
#include <igl/read_triangle_mesh.h>

bool load_verts(std::string const& file_name, Eigen::MatrixXd& V, double scaling_factor,
	std::vector<std::vector<int>> & polygons)
{
	std::vector<std::vector<double>> verts;

	if (!igl::readOBJ(file_name, verts, polygons))
	{
		std::cerr << "Failed to load " << file_name << "!\n";
		return false;
	}

	if (scaling_factor != 1.0)
	{
		scale_verts(verts, scaling_factor);
	}

	V.resize(verts.size(), 3);

	for (int i = 0; i < verts.size(); ++i)
	{
		auto const& vert = verts[i];
		V.row(i) = Eigen::Vector3d(vert[0], vert[1], vert[2]);
	}

	return true;
}

bool load_mesh(std::string const& file_name, Eigen::MatrixXd& V, Eigen::MatrixXi& T, double scaling_factor)
{
	if (file_name.substr(file_name.size() - 4, 4).compare(".msh") == 0)
	{
		Eigen::MatrixXi Triangles;
		Eigen::VectorXi TriTags, TetTags;
		if (!igl::readMSH(file_name, V, Triangles, T, TriTags, TetTags))
		{
			std::cerr << "Failed to load " << file_name << "\n";
			return false;
		}

		if (scaling_factor != 1.0)
		{
			V *= scaling_factor;
		}

	}
	else if (file_name.substr(file_name.size() - 4, 4).compare(".obj") == 0)
	{
		std::vector<std::vector<int>> polygons;

		bool uses_quads = false;

		if (!load_verts(file_name, V, scaling_factor, polygons))
		{	
			return false;
		}

		for (auto&& poly : polygons)
		{
			if (poly.size() == 4)
			{
				uses_quads = true;
				break;
			}
		}

		T.resize(polygons.size(), uses_quads ? 4 : 3);

		for (int i = 0; i < polygons.size(); ++i)
		{
			auto const& polygon = polygons[i];
			assert(polygon.size() == 3 || (uses_quads && polygon.size() == 4));
			if (polygon.size() == 3)
			{
				T.row(i) = Eigen::Vector3i(polygon[0], polygon[1], polygon[2]);
			}
			else if (polygon.size() == 4)
			{
				T.row(i) = Eigen::Vector4i(polygon[0], polygon[1], polygon[2], polygon[3]);
			}
			else 
			{
				std::cerr << "Unsupported polygon type!\n";
			}
		}

	}
	else
	{
		std::cerr << "Unsupported input format\n";
		return false;
	}

	return true;
}

void load_cage(Eigen::MatrixXd& V, const std::vector<std::vector<int>>& polys, Eigen::VectorXi& P, Eigen::MatrixXi& CF,
	bool triangulate_quads, Eigen::MatrixXd * V_embedding /*= nullptr*/, bool find_offset /*= false*/)
{
	int cage_vertices_offset = 0;

	if (V_embedding)
	{
		if (find_offset)
		{
			auto verices_equal = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b)
			{
				auto const dist = (a - b).norm();
				return dist < 1.e-6;
			};

			const Eigen::Vector3d first_cage_vert = V.row(0);
			bool found = false;
			for (int i = 0; i < V_embedding->rows(); ++i)
			{
				const Eigen::Vector3d embedding_vert = V_embedding->row(i);
				if (verices_equal(first_cage_vert, embedding_vert))
				{
					found = true;
					cage_vertices_offset = i;
					break;
				}
			}
			if (found && verbosity)
			{
				std::cout << "Found cage verts in embedding with an offset of " << cage_vertices_offset << "\n";
			}
			else
			{
				std::cerr << "Could not find cage verts in embedding\n";
				return;
			}
		}
		for (int i = 0; i < V.rows(); ++i)
		{
			V.row(i) = V_embedding->row(i + cage_vertices_offset);
		}
	}

	P.resize(V.rows());
	P.fill(0);
	for (int i = 0; i < V.rows(); ++i)
	{
		P(i) = i;
	}

	unsigned int numTriangles = 0, numQuads = 0;

	for (int i = 0; i < polys.size(); ++i)
	{
		auto const numVertsPoly = polys[i].size();
		assert(numVertsPoly == 3 || numVertsPoly == 4);

		if (numVertsPoly == 3)
		{
			++numTriangles;
		}
		else // Found a quad
		{
			if (triangulate_quads)
			{
				numTriangles += 2u;
			}
			else
			{
				++numQuads;
			}
		}
	}

	if (triangulate_quads && verbosity)
	{
		std::cout << "Cage Triangles: " << numTriangles << " Cage Vertices: " << V.rows() << "\n";
	}
	else if (verbosity)
	{
		std::cout << "Cage Triangles: " << numTriangles << " Cage Quads " << numQuads << " Cage Vertices: " << V.rows() << "\n";
	}

	CF.resize(triangulate_quads ? numTriangles : numTriangles + numQuads, numQuads ? 4 : 3);

	int row_idx = 0;
	for (int i = 0; i < polys.size(); ++i)
	{
		auto const& poly = polys[i];
		auto const numVertsPoly = poly.size();

		if (numVertsPoly == 3)
		{
			if (numQuads)
			{
				CF.row(row_idx++) = Eigen::Vector4i(poly[0], poly[1], poly[2], -1);
			}
			else
			{
				CF.row(row_idx++) = Eigen::Vector3i(poly[0], poly[1], poly[2]);
			}

		}
		else // quad
		{
			if (triangulate_quads)
			{
				CF.row(row_idx++) = Eigen::Vector3i(poly[0], poly[1], poly[2]);
				CF.row(row_idx++) = Eigen::Vector3i(poly[0], poly[2], poly[3]);
			}
			else
			{
				CF.row(row_idx++) = Eigen::Vector4i(poly[0], poly[1], poly[2], poly[3]);
			}
		}
	}
}

bool load_cage(std::string const& file_name, Eigen::MatrixXd& V,
	Eigen::VectorXi & P, Eigen::MatrixXi & CF, double scaling_factor, bool triangulate_quads,
	Eigen::MatrixXd * V_embedding /*= nullptr*/, bool find_offset /*= false*/)
{
	std::vector<std::vector<int>> polys;

	if (!load_verts(file_name, V, scaling_factor, polys))
	{
		return false;
	}

	load_cage(V, polys, P, CF, triangulate_quads, V_embedding, find_offset);

	return true;
}

struct Vector3dHash
{
	std::size_t operator()(const Eigen::Vector3d& v) const
	{
		double scale = 1e8;
		int x = static_cast<int>(std::round(v[0] * scale));
		int y = static_cast<int>(std::round(v[1] * scale));
		int z = static_cast<int>(std::round(v[2] * scale));
		return std::hash<int>()(x) ^ std::hash<int>()(y) ^ std::hash<int>()(z);
	}

	bool operator()(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) const
	{
		double tolerance = 1e-8;
		return (std::abs(v1[0] - v2[0]) < tolerance &&
			std::abs(v1[1] - v2[1]) < tolerance &&
			std::abs(v1[2] - v2[2]) < tolerance);
	}
};
//BGC related code begin
bool load_bezier_surface_cage(const std::string& file_name, int dim, Eigen::MatrixXd& V, Eigen::MatrixXi& CF, std::vector<int>& num_vertices_per_line)
{
	num_vertices_per_line.clear();
	int numControlPointsBezierTriangle = (dim + 2) * (dim + 1) / 2;
	int nuCcontrolPointsBezierQuad = (dim + 1) * (dim + 1);

	std::ifstream file(file_name);
	if (!file.is_open()) {
		std::cerr << "Cannot open file: " << file_name << std::endl;
		return false;
	}

	std::vector<Eigen::Vector3d> all_points;
	std::vector<std::vector<int>> surface_indices;

	std::unordered_map<Eigen::Vector3d, int, Vector3dHash> point_to_index;

	std::string line;
	while (std::getline(file, line))
	{
		std::istringstream ss(line);
		std::vector<Eigen::Vector3d> points;
		Eigen::Vector3d point;

		while (ss >> point[0] >> point[1] >> point[2])
		{
			points.push_back(point);

			if (point_to_index.find(point) == point_to_index.end())
			{
				int index = all_points.size();
				point_to_index[point] = index;
				all_points.push_back(point);
			}
		}
		int num_control_points = points.size();
		if (num_control_points != numControlPointsBezierTriangle && num_control_points != nuCcontrolPointsBezierQuad)
		{
			std::cerr << "Invalid number of control points in line: " << line << std::endl;
			return false;
		}

		std::vector<int> indices(num_control_points);
		num_vertices_per_line.push_back(num_control_points);
		for (int i = 0; i < num_control_points; ++i)
		{
			indices[i] = point_to_index[points[i]];
		}
		surface_indices.push_back(indices);
	}

	file.close();

	// Resize the output matrices based on the collected data
	V.resize(all_points.size(), 3);
	CF.resize(surface_indices.size(), nuCcontrolPointsBezierQuad);

	for (size_t i = 0; i < all_points.size(); ++i)
	{
		V.row(i) = all_points[i].transpose();
	}

	for (size_t i = 0; i < surface_indices.size(); ++i)
	{
		CF.row(i) = Eigen::VectorXi::Map(surface_indices[i].data(), surface_indices[i].size());
	}

	return true;
}


void computePointCross(Eigen::MatrixXd& V, Eigen::MatrixXi& CF, std::vector<Eigen::MatrixXd>& patch_point_cross, std::vector<int>& num_vertices_per_line)
{
	std::vector<std::vector<int>> patch_indices;
	patch_indices.reserve(CF.rows());

	for (int i = 0; i < CF.rows(); ++i) {
		std::vector<int> patch;
		patch.reserve(CF.cols());
		for (int j = 0; j < CF.cols(); ++j) {
			patch.push_back(CF(i, j));
		}
		patch_indices.push_back(patch);
	}


	int count = 0;
	for (const auto& indices : patch_indices)
	{
		int numVertices = num_vertices_per_line[count];
		Eigen::MatrixXd all_points_this_patch(numVertices, 3);
		for (int i = 0; i < numVertices; ++i)
		{
			int vertexIndex = CF(count, i);
			all_points_this_patch.row(i) = V.row(vertexIndex);
		}

		Eigen::MatrixXd cross_patch = Eigen::MatrixXd::Zero((int)(num_vertices_per_line[count] * (num_vertices_per_line[count] - 1) / 2), 3);
		int cross_index = 0;
		for (int i = 0; i < numVertices; ++i)
		{
			for (int j = i + 1; j < numVertices; ++j)
			{
				Eigen::Vector3d vec1 = all_points_this_patch.row(i).transpose();
				Eigen::Vector3d vec2 = all_points_this_patch.row(j).transpose();
				cross_patch.row(cross_index) = vec1.cross(vec2).transpose();
				cross_index++;
			}
		}
		patch_point_cross.push_back(cross_patch);

		count++;
	}
}



int returnTriangleIndex(int i, int j, int dim)
{
	return (int)(((dim - i) * (1 + (dim - i))) / 2 + (dim - i - j));
}

int returnQuadIndex(int i, int j, int dim)
{
	return (int)(j * (dim + 1) + i);
}

void computeNormalBezier(Eigen::MatrixXd& V, Eigen::MatrixXi& CF, std::vector<Eigen::MatrixXd>& patchNormals, std::vector<int>& num_vertices_per_line, int dim)
{
	std::vector<std::vector<int>> patch_indices;
	patch_indices.reserve(CF.rows());

	for (int i = 0; i < CF.rows(); ++i) {
		std::vector<int> patch;
		patch.reserve(CF.cols());
		for (int j = 0; j < CF.cols(); ++j) {
			patch.push_back(CF(i, j));
		}
		patch_indices.push_back(patch);
	}

	int count = 0;
	int pointNumBezierBezierTriangle = (dim + 1) * (dim + 2) / 2.0;
	int pointNumBezierBezierQuad = (dim + 1) * (dim + 1);

	for (const auto& indices : patch_indices)
	{
		int numVertices = num_vertices_per_line[count];
		Eigen::MatrixXd allPointsThisPatch(numVertices, 3);
		for (int i = 0; i < numVertices; ++i)
		{
			int vertexIndex = CF(count, i);
			allPointsThisPatch.row(i) = V.row(vertexIndex);
		}

		if (num_vertices_per_line[count] == pointNumBezierBezierTriangle)
		{
			Eigen::MatrixXd normalsPatch = Eigen::MatrixXd::Zero(pointNumBezierBezierTriangle, 3);
			Eigen::Vector3d v1, v2;
			for (int i = dim; i >= 0; i--)
			{
				for (int j = dim - i; j >= 0; j--)
				{
					Eigen::Vector3d sumNormal = Eigen::Vector3d::Zero();
					double sumCount = 0.0;
					int k = dim - i - j;
					Eigen::Vector3d bijk = allPointsThisPatch.row(returnTriangleIndex(i, j, dim)).transpose();
					if (k - 1 >= 0 && i + 1 <= dim && j + 1 <= dim)
					{
						v1 = allPointsThisPatch.row(returnTriangleIndex(i + 1, j, dim)).transpose() - bijk;
						v2 = allPointsThisPatch.row(returnTriangleIndex(i, j + 1, dim)).transpose() - bijk;
						sumNormal += v1.cross(v2);
						sumCount += 1.0;
					}
					if (k - 1 >= 0 && i - 1 >= 0 && j + 1 <= dim)
					{
						v1 = allPointsThisPatch.row(returnTriangleIndex(i, j + 1, dim)).transpose() - bijk;
						v2 = allPointsThisPatch.row(returnTriangleIndex(i - 1, j + 1, dim)).transpose() - bijk;
						sumNormal += v1.cross(v2);
						sumCount += 1.0;
					}
					if (i - 1 >= 0 && j + 1 <= dim && k + 1 <= dim)
					{
						v1 = allPointsThisPatch.row(returnTriangleIndex(i - 1, j + 1, dim)).transpose() - bijk;
						v2 = allPointsThisPatch.row(returnTriangleIndex(i - 1, j, dim)).transpose() - bijk;
						sumNormal += v1.cross(v2);
						sumCount += 1.0;
					}
					if (i - 1 >= 0 && j - 1 >= 0 && k + 1 <= dim)
					{
						v1 = allPointsThisPatch.row(returnTriangleIndex(i - 1, j, dim)).transpose() - bijk;
						v2 = allPointsThisPatch.row(returnTriangleIndex(i, j - 1, dim)).transpose() - bijk;
						sumNormal += v1.cross(v2);
						sumCount += 1.0;
					}
					if (j - 1 >= 0 && i + 1 <= dim && k + 1 <= dim)
					{
						v1 = allPointsThisPatch.row(returnTriangleIndex(i, j - 1, dim)).transpose() - bijk;
						v2 = allPointsThisPatch.row(returnTriangleIndex(i + 1, j - 1, dim)).transpose() - bijk;
						sumNormal += v1.cross(v2);
						sumCount += 1.0;
					}
					if (j - 1 >= 0 && k - 1 >= 0 && i + 1 <= dim)
					{
						v1 = allPointsThisPatch.row(returnTriangleIndex(i + 1, j - 1, dim)).transpose() - bijk;
						v2 = allPointsThisPatch.row(returnTriangleIndex(i + 1, j, dim)).transpose() - bijk;
						sumNormal += v1.cross(v2);
						sumCount += 1.0;
					}
					normalsPatch.row(returnTriangleIndex(i, j, dim)) = sumNormal / sumCount * dim * dim;
				}
			}
			patchNormals.push_back(normalsPatch);
		}
		else
		{
			Eigen::MatrixXd normalsPatch = Eigen::MatrixXd::Zero(pointNumBezierBezierQuad, 3);
			Eigen::Vector3d v1, v2;
			for (int j = 0; j <= dim; j++)
			{
				for (int i = 0; i <= dim; i++)
				{
					Eigen::Vector3d sumNormal = Eigen::Vector3d::Zero();
					double sumCount = 0.0;
					Eigen::Vector3d bij = allPointsThisPatch.row(returnQuadIndex(i, j, dim)).transpose();
					if (j + 1 <= dim && i - 1 >= 0)
					{
						v1 = allPointsThisPatch.row(returnQuadIndex(i, j + 1, dim)).transpose() - bij;
						v2 = allPointsThisPatch.row(returnQuadIndex(i - 1, j, dim)).transpose() - bij;
						sumNormal += v1.cross(v2);
						sumCount += 1.0;
					}
					if (i - 1 >= 0 && j - 1 >= 0)
					{
						v1 = allPointsThisPatch.row(returnQuadIndex(i - 1, j, dim)).transpose() - bij;
						v2 = allPointsThisPatch.row(returnQuadIndex(i, j - 1, dim)).transpose() - bij;
						sumNormal += v1.cross(v2);
						sumCount += 1.0;
					}
					if (j - 1 >= 0 && i + 1 <= dim)
					{
						v1 = allPointsThisPatch.row(returnQuadIndex(i, j - 1, dim)).transpose() - bij;
						v2 = allPointsThisPatch.row(returnQuadIndex(i + 1, j, dim)).transpose() - bij;
						sumNormal += v1.cross(v2);
						sumCount += 1.0;
					}
					if (i + 1 <= dim && j + 1 <= dim)
					{
						v1 = allPointsThisPatch.row(returnQuadIndex(i + 1, j, dim)).transpose() - bij;
						v2 = allPointsThisPatch.row(returnQuadIndex(i, j + 1, dim)).transpose() - bij;
						sumNormal += v1.cross(v2);
						sumCount += 1.0;
					}
					normalsPatch.row(returnQuadIndex(i, j, dim)) = sumNormal / sumCount * dim * dim;
				}
			}
			patchNormals.push_back(normalsPatch);
		}
		count++;
	}
}
//BGC related code end