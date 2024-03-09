#include <cagedeformations/LoadMesh.h>
#include <cagedeformations/globals.h>

#include <vector>
#include <iostream>

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