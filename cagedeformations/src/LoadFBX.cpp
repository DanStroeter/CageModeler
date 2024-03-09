#include <cagedeformations/LoadFBX.h>
#include <cagedeformations/globals.h>

#include <vector>
#include <OpenFBX/ofbx.h>
#include <iostream>
#include <igl/vector_area_matrix.h>

void load_mesh(ofbx::IScene* scene, const int mesh_idx, Eigen::MatrixXd& V_model, Eigen::MatrixXi& T_model)
{
	const int offset_vert_idx = V_model.rows();
	int cur_vert_idx = offset_vert_idx;
	const int numVerts = scene->getMesh(mesh_idx)->getGeometry()->getGeometryData().getPositions().values_count;
	V_model.conservativeResize(cur_vert_idx + numVerts, 3);

	auto const indices = scene->getMesh(mesh_idx)->getGeometry()->getGeometryData().getPositions().indices;

	for (int i = 0; i < numVerts; ++i)
	{
		auto const vertex = scene->getMesh(mesh_idx)->getGeometry()->getGeometryData().getPositions().values[i];
		V_model.row(cur_vert_idx++) = Eigen::Vector3d(vertex.x, vertex.y, vertex.z);
	}

	const int numPartitions = scene->getMesh(mesh_idx)->getGeometry()->getGeometryData().getPartitionCount();

	std::cout << "Partitions = " << scene->getMesh(mesh_idx)->getGeometry()->getGeometryData().getPartitionCount() << "\n";

	int numTotalTriangles = 0;

	for (int partition = 0; partition < numPartitions; ++partition)
	{
		numTotalTriangles += scene->getMesh(mesh_idx)->getGeometry()->getGeometryData().getPartition(partition).triangles_count;
	}

	int cur_row_idx = T_model.rows();
	T_model.conservativeResize(cur_row_idx + numTotalTriangles, 3);

	for (int partition = 0; partition < numPartitions; ++partition)
	{
		auto const polygons = scene->getMesh(mesh_idx)->getGeometry()->getGeometryData().getPartition(partition).polygons;
		int numPolys = scene->getMesh(mesh_idx)->getGeometry()->getGeometryData().getPartition(partition).polygon_count;

		for (int poly_idx = 0; poly_idx < numPolys; ++poly_idx)
		{
			auto const poly = polygons[poly_idx];
			const int numTriangles = poly.vertex_count - 2;
			std::vector<int> vertices_poly_triangulated(3 * numTriangles);

			triangulate(scene->getMesh(mesh_idx)->getGeometry()->getGeometryData(), poly, vertices_poly_triangulated.data());
			for (int tri_idx = 0; tri_idx < numTriangles; ++tri_idx)
			{
				Eigen::Vector3i triangle_indices(indices[vertices_poly_triangulated[3 * tri_idx]] + offset_vert_idx,
					indices[vertices_poly_triangulated[3 * tri_idx + 1]] + offset_vert_idx,
					indices[vertices_poly_triangulated[3 * tri_idx + 2]] + offset_vert_idx);

				T_model.row(cur_row_idx++) = triangle_indices;
			}
		}
	}
}

bool load_fbx_file(const std::string& fbxFile, Eigen::MatrixXd & V_model, Eigen::MatrixXi & T_model, Eigen::MatrixXd& V_cage, Eigen::MatrixXi& T_cage)
{
	auto fbx_file = fopen(fbxFile.c_str(), "rb");
	if (!fbx_file)
	{
		std::cerr << "Unable to open " << fbxFile << "\n";
		return false;
	}

	fseek(fbx_file, 0, SEEK_END);
	long file_size = ftell(fbx_file);
	fseek(fbx_file, 0, SEEK_SET);

	std::vector<ofbx::u8> content(file_size);

	auto const data_size = fread(content.data(), 1, file_size, fbx_file);

	if (!data_size)
	{
		return false;
	}

	ofbx::LoadFlags flags =
		//		ofbx::LoadFlags::IGNORE_MODELS |
		ofbx::LoadFlags::IGNORE_BLEND_SHAPES |
		ofbx::LoadFlags::IGNORE_CAMERAS |
		ofbx::LoadFlags::IGNORE_LIGHTS |
		ofbx::LoadFlags::IGNORE_TEXTURES |
		ofbx::LoadFlags::IGNORE_SKIN |
		ofbx::LoadFlags::IGNORE_BONES |
		ofbx::LoadFlags::IGNORE_PIVOTS |
		ofbx::LoadFlags::IGNORE_MATERIALS |
		ofbx::LoadFlags::IGNORE_POSES |
		ofbx::LoadFlags::IGNORE_VIDEOS |
		ofbx::LoadFlags::IGNORE_LIMBS |
		//		ofbx::LoadFlags::IGNORE_MESHES |
		ofbx::LoadFlags::IGNORE_ANIMATIONS;

	auto const scene = ofbx::load(content.data(), file_size, 0);

	if (verbosity)
	{
		std::cout << "Mesh Count is " << scene->getMeshCount() << "\n";
	}

	for (int mesh_idx = 0; mesh_idx < scene->getMeshCount(); ++mesh_idx)
	{
		std::string mesh_name(scene->getMesh(mesh_idx)->name);
		std::transform(mesh_name.begin(), mesh_name.end(), mesh_name.begin(),
			[](unsigned char c) { return std::tolower(c); });
		if (verbosity)
		{
			std::cout << "Loading " << mesh_name << "\n";
		}
		if (mesh_name.find("cage") != std::string::npos)
		{
			load_mesh(scene, mesh_idx, V_cage, T_cage);
		}
		else
		{
			load_mesh(scene, mesh_idx, V_model, T_model);
		}
	}

	return true;
}