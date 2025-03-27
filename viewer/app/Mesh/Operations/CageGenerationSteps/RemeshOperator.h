#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <omp.h>
#include <Mesh/Operations/CageGenerationSteps/Utils.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
//#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
//#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>

//#include <CGAL/Polygon_mesh_processing/intersection.h>
//#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
//#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
//#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
//#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
//#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
//#include <CGAL/Polygon_mesh_processing/compute_normal.h>
//#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>

#define BUFFER_OFFSET(offset) ((GLvoid*)(offset))

typedef CGAL::Exact_predicates_exact_constructions_kernel			Exact_Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel			Inexact_Kernel;
typedef CGAL::Polyhedron_3<Inexact_Kernel>							Inexact_Polyhedron;
typedef CGAL::Polyhedron_3<Exact_Kernel>							Exact_Polyhedron;
typedef Exact_Kernel::Point_3										ExactPoint;
typedef Inexact_Kernel::Point_3										InexactPoint;
typedef Exact_Kernel::Vector_3										ExactVector;
typedef CGAL::Surface_mesh<InexactPoint>							Mesh;
typedef CGAL::Surface_mesh<ExactPoint>								ExactMesh;
//typedef CGAL::AABB_face_graph_triangle_primitive<Exact_Polyhedron>	Primitive;
//typedef CGAL::AABB_traits<Exact_Kernel, Primitive>					Traits;
//typedef CGAL::AABB_tree<Traits>										Tree;
//typedef CGAL::Side_of_triangle_mesh<Exact_Polyhedron, Exact_Kernel>	Point_inside;
typedef std::vector<bool> VOXEL_GRID;
//typedef std::vector<VOXEL_GRID>	MIPMAP_TYPE;

namespace PMP = CGAL::Polygon_mesh_processing;

#include <vcg/complex/complex.h>

// io
#include <wrap/io_trimesh/import_obj.h>
#include <wrap/io_trimesh/import_off.h>
#include <wrap/io_trimesh/export.h>

// local optimization
#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/smooth.h>

using namespace vcg;
using namespace tri;

class MyVertex;
class MyEdge;
class MyFace;

struct MyUsedTypes : public UsedTypes<Use<MyVertex>::AsVertexType, Use<MyEdge>::AsEdgeType, Use<MyFace>::AsFaceType> {};

class MyVertex : public vcg::Vertex< MyUsedTypes,
	vertex::VFAdj,
	vertex::Coord3f,
	vertex::Normal3f,
	vertex::Mark,
	vertex::BitFlags  > {
public:
	vcg::math::Quadric<double>& Qd() { return q; }
	bool is_feature() { return is_feature_; }
	void set_is_feature(bool is_feature) { is_feature_ = is_feature; }
private:
	math::Quadric<double> q;
	bool is_feature_;
};

class MyEdge : public Edge< MyUsedTypes> {};

typedef BasicVertexPair<MyVertex> VertexPair;

class MyFace : public vcg::Face< MyUsedTypes,
	face::VFAdj,
	face::VertexRef,
	face::BitFlags > {};

class MyMesh : public vcg::tri::TriMesh<std::vector<MyVertex>, std::vector<MyFace> > {};
typedef typename MyMesh::ScalarType ScalarType;
typedef typename MyMesh::CoordType CoordType;
typedef MyMesh::VertexType::EdgeType EdgeType;
typedef typename MyMesh::VertexIterator VertexIterator;
typedef typename MyMesh::VertexPointer VertexPointer;
typedef typename MyMesh::FaceIterator FaceIterator;
typedef typename MyMesh::FacePointer FacePointer;

class MyTriEdgeCollapse : public vcg::tri::TriEdgeCollapseQuadric< MyMesh, VertexPair, MyTriEdgeCollapse, QInfoStandard<MyVertex>  > {
public:
	typedef  vcg::tri::TriEdgeCollapseQuadric< MyMesh, VertexPair, MyTriEdgeCollapse, QInfoStandard<MyVertex>  > TECQ;
	typedef  MyMesh::VertexType::EdgeType EdgeType;
	inline MyTriEdgeCollapse(const VertexPair& p, int i, BaseParameterClass* pp) :TECQ(p, i, pp) {}
};

struct PointHash {
	std::size_t operator()(const ExactPoint& p) const {
		auto h1 = std::hash<double>{}(CGAL::to_double(p.x()));
		auto h2 = std::hash<double>{}(CGAL::to_double(p.y()));
		auto h3 = std::hash<double>{}(CGAL::to_double(p.z()));
		return h1 ^ (h2 << 1) ^ (h3 << 2);
	}
};

class RemeshOperator: public Utils
{
public:
	RemeshOperator(int resolution, float cell_size) : _resolution(resolution), _cellSize(cell_size)
	{
	
	}
	~RemeshOperator() {}

private:

	int _resolution;
	float _cellSize;

	std::array<ExactPoint, 8> compute_voxel_vertices(
		const ExactPoint& origin,
		const std::array<ExactVector, 3>& voxel_strides)
	{
		return { origin,
				origin + voxel_strides[0],
				origin + voxel_strides[1],
				origin + voxel_strides[2],
				origin + voxel_strides[0] + voxel_strides[1],
				origin + voxel_strides[0] + voxel_strides[2],
				origin + voxel_strides[1] + voxel_strides[2],
				origin + voxel_strides[0] + voxel_strides[1] + voxel_strides[2] };
	}
	void add_voxel_faces(
		ExactMesh& mesh,
		const std::array<ExactMesh::Vertex_index, 8>& mesh_vertices,
		const std::array<bool, 6>& exposed_faces)
	{
		static const std::array<std::array<int, 4>, 6> faces = { {
			{{0, 3, 6, 2}},  // -X
			{{1, 4, 7, 5}},  // +X
			{{1, 5, 3, 0}},  // -Y
			{{2, 6, 7, 4}},  // +Y
			{{1, 0, 2, 4}},  // -Z
			{{3, 5, 7, 6}}   // +Z
		} };

		for (int i = 0; i < 6; ++i) {
			if (exposed_faces[i]) {
				mesh.add_face(mesh_vertices[faces[i][0]], mesh_vertices[faces[i][1]], mesh_vertices[faces[i][2]]);
				mesh.add_face(mesh_vertices[faces[i][2]], mesh_vertices[faces[i][3]], mesh_vertices[faces[i][0]]);
			}
		}
	}

public:
	ExactMesh ExtractSurface(
		VOXEL_GRID& grid, std::array<float, 3>& bbox_min)
	{
		std::array<ExactVector, 3> voxel_strides = { ExactVector(_cellSize, 0, 0),
	ExactVector(0, _cellSize, 0), ExactVector(0, 0, _cellSize) };
		const std::array<std::array<int, 3>, 6> neighbors = { {
			{{-1, 0, 0}}, {{1, 0, 0}},  // X 
			{{0, -1, 0}}, {{0, 1, 0}},  // Y 
			{{0, 0, -1}}, {{0, 0, 1}}   // Z
		} };

		ExactMesh output_mesh;
		size_t nx = _resolution, ny = _resolution, nz = _resolution;

		std::unordered_map<ExactPoint, ExactMesh::Vertex_index, PointHash> vertex_map;
		ExactPoint origin(bbox_min[0], bbox_min[1], bbox_min[2]);
		for (int x = 0; x < nx; ++x) {
			for (int y = 0; y < ny; ++y) {
				for (int z = 0; z < nz; ++z) {
					int idx = coords_to_voxel_idx(x, y, z, _resolution);
					if (!grid[idx]) continue;

					std::array<ExactPoint, 8> voxel_vertices =
						compute_voxel_vertices(origin + x * voxel_strides[0] + y * voxel_strides[1] + z * voxel_strides[2], voxel_strides);

					std::array<ExactMesh::Vertex_index, 8> mesh_vertices;
					for (int i = 0; i < 8; ++i) {
						auto it = vertex_map.find(voxel_vertices[i]);
						if (it == vertex_map.end()) {

							ExactMesh::Vertex_index v = output_mesh.add_vertex(voxel_vertices[i]);
							mesh_vertices[i] = v;
							vertex_map[voxel_vertices[i]] = v;
						}
						else {
							mesh_vertices[i] = it->second;
						}
					}

					std::array<bool, 6> exposed_faces = { true, true, true, true, true, true };
					for (int i = 0; i < 6; ++i) {
						int nx = x + neighbors[i][0];
						int ny = y + neighbors[i][1];
						int nz = z + neighbors[i][2];
						int n_idx = coords_to_voxel_idx(nx, ny, nz, _resolution);
						if (nx >= 0 && nx < _resolution &&
							ny >= 0 && ny < _resolution &&
							nz >= 0 && nz < _resolution &&
							grid[n_idx]) {
							exposed_faces[i] = false;
						}
					}

					add_voxel_faces(output_mesh, mesh_vertices, exposed_faces);
				}
			}
		}

		return output_mesh;
	}

	void Decimate(MyMesh& vcg_mesh, const int smoothIterations, const int targetNumFaces) {

		tri::Smooth<MyMesh>::VertexCoordLaplacianHC(vcg_mesh, smoothIterations);

		TriEdgeCollapseQuadricParameter qparams;
		qparams.QualityThr = .3;

		qparams.QualityCheck = true;
		qparams.NormalCheck = true;
		qparams.OptimalPlacement = true;
		qparams.ScaleIndependent = true;
		qparams.PreserveTopology = true;

		bool CleaningFlag = true;
		if (CleaningFlag) {
			int dup = tri::Clean<MyMesh>::RemoveDuplicateVertex(vcg_mesh);
			int unref = tri::Clean<MyMesh>::RemoveUnreferencedVertex(vcg_mesh);
		}

		vcg::tri::UpdateBounding<MyMesh>::Box(vcg_mesh);

		// decimator initialization
		vcg::LocalOptimization<MyMesh> DeciSession(vcg_mesh, &qparams);

		DeciSession.Init<MyTriEdgeCollapse>();
		DeciSession.SetTargetSimplices(targetNumFaces);
		DeciSession.SetTimeBudget(0.5f);
		DeciSession.SetTargetOperations(100000);

		while (DeciSession.DoOptimization() && vcg_mesh.fn > targetNumFaces)

		if (CleaningFlag) {
			int dup = tri::Clean<MyMesh>::RemoveDuplicateVertex(vcg_mesh);
			int unref = tri::Clean<MyMesh>::RemoveUnreferencedVertex(vcg_mesh);
			int deg_face = tri::Clean<MyMesh>::RemoveDegenerateFace(vcg_mesh);
			int dup_face = tri::Clean<MyMesh>::RemoveDuplicateFace(vcg_mesh);
		}
	}
};