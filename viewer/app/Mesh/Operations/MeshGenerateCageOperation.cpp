#include<Mesh/Operations/MeshGenerateCageOperation.h>
#include <cagedeformations/LoadMesh.h>
#include <cagedeformations/GreenCoordinates.h>
#include <iostream>
#include <utility>
#include <array>
#include <fstream>
#include <algorithm>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <bitset>
#include <omp.h>
#include <ctime>
#define CGAL_EIGEN3_ENABLED

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <CGAL/Optimal_bounding_box/oriented_bounding_box.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Polyhedral_envelope_filter.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_filter.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_stop_predicate.h>
#include <boost/optional/optional_io.hpp>


typedef CGAL::Exact_predicates_exact_constructions_kernel			Exact_Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel			Inexact_Kernel;
typedef CGAL::Polyhedron_3<Inexact_Kernel>							Inexact_Polyhedron;
typedef CGAL::Polyhedron_3<Exact_Kernel>							Exact_Polyhedron;
typedef CGAL::Nef_polyhedron_3<Exact_Kernel>						Nef_polyhedron;
typedef Exact_Kernel::Point_3										ExactPoint;
typedef Inexact_Kernel::Point_3										InexactPoint;
typedef Inexact_Kernel::Triangle_3									Triangle;
typedef Exact_Kernel::Vector_3										ExactVector;
typedef CGAL::Surface_mesh<InexactPoint>							Mesh;
typedef CGAL::Surface_mesh<ExactPoint>								ExactMesh;
typedef CGAL::AABB_face_graph_triangle_primitive<Exact_Polyhedron>	Primitive;
typedef CGAL::AABB_traits<Exact_Kernel, Primitive>					Traits;
typedef CGAL::AABB_tree<Traits>										Tree;
typedef CGAL::Side_of_triangle_mesh<Exact_Polyhedron, Exact_Kernel>	Point_inside;

typedef boost::graph_traits<Inexact_Polyhedron>::edge_descriptor      edge_descriptor;
typedef boost::graph_traits<Inexact_Polyhedron>                             GraphTraits;
typedef typename GraphTraits::halfedge_descriptor                           halfedge_descriptor;
typedef typename GraphTraits::vertex_descriptor                             vertex_descriptor;
typedef Exact_Polyhedron::Vertex_handle Vertex;
typedef Exact_Polyhedron::Face_handle Face;
typedef std::vector<bool> VOXEL_GRID;
typedef std::vector<VOXEL_GRID>	MIPMAP_TYPE;

namespace PMP = CGAL::Polygon_mesh_processing;
namespace SMS = CGAL::Surface_mesh_simplification;


// save diagnostic state
#pragma GCC diagnostic push 

// turn off the specific warning. Can also use "-Wall"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wmissing-template-arg-list-after-template-kw"


// for decimation

// stuff to define the mesh
#include <vcg/complex/complex.h>

//#define GLEW_STATIC 1
#include <GL/glew.h>
#include <GLFW/glfw3.h>
//#include <GL/glut.h>
#define BUFFER_OFFSET(offset) ((GLvoid*)(offset))

// io
#include <wrap/io_trimesh/import_obj.h>
#include <wrap/io_trimesh/import_off.h>
#include <wrap/io_trimesh/export.h>

// local optimization
#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/smooth.h>

//#include <vcg/export_off.h> 

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

//#define BASE_RESOLUTION 32
int BASE_RESOLUTION = 32;
float SE_SIZE = 2.f;
typedef struct {
	int level;
	unsigned int pos;
} Node;


unsigned int coords_to_voxel_idx(unsigned int x_idx, unsigned int y_idx, unsigned int z_idx, std::array<unsigned int, 3> numVoxels)
{
	return z_idx + y_idx * numVoxels[2] + x_idx * numVoxels[2] * numVoxels[1];
}

unsigned int coords_to_voxel_idx(unsigned int x_idx, unsigned int y_idx, unsigned int z_idx, int numVoxels)
{
	return z_idx + y_idx * numVoxels + x_idx * numVoxels * numVoxels;
}

void convert_voxel_idx_to_coords(unsigned int idx, unsigned int numVoxels, unsigned int& x_idx, unsigned int& y_idx, unsigned int& z_idx)
{
	x_idx = idx / (numVoxels * numVoxels);
	auto const w = idx % (numVoxels * numVoxels);
	y_idx = w / numVoxels;
	z_idx = w % numVoxels;
}

std::array<ExactPoint, 8> calc_voxel_points(unsigned int idx, std::array<unsigned int, 3> numVoxels, ExactPoint min_point,
	const std::array<ExactVector, 3>& voxel_strides, bool* new_scanline = nullptr)
{
	unsigned int x_idx, y_idx, z_idx;

	convert_voxel_idx_to_coords(idx, numVoxels[0], x_idx, y_idx, z_idx);

	if (new_scanline)
	{
		*new_scanline = z_idx == 0;
	}

	return {
		min_point + x_idx * voxel_strides[0] + y_idx * voxel_strides[1] + z_idx * voxel_strides[2],
		min_point + (x_idx + 1u) * voxel_strides[0] + y_idx * voxel_strides[1] + z_idx * voxel_strides[2],
		min_point + x_idx * voxel_strides[0] + (y_idx + 1u) * voxel_strides[1] + z_idx * voxel_strides[2],
		min_point + (x_idx + 1u) * voxel_strides[0] + (y_idx + 1u) * voxel_strides[1] + z_idx * voxel_strides[2],
		min_point + x_idx * voxel_strides[0] + y_idx * voxel_strides[1] + (z_idx + 1u) * voxel_strides[2],
		min_point + (x_idx + 1u) * voxel_strides[0] + y_idx * voxel_strides[1] + (z_idx + 1u) * voxel_strides[2],
		min_point + x_idx * voxel_strides[0] + (y_idx + 1u) * voxel_strides[1] + (z_idx + 1u) * voxel_strides[2],
		min_point + (x_idx + 1u) * voxel_strides[0] + (y_idx + 1u) * voxel_strides[1] + (z_idx + 1u) * voxel_strides[2]
	};
}



// make a voxel with 4 tetrahedron?
void calc_voxel_from_idx_tets(unsigned int idx, std::array<unsigned int, 3> numVoxels, ExactPoint min_point,
	const std::array<ExactVector, 3>& voxel_strides, Exact_Polyhedron& voxel, bool* new_scanline)
{
	auto const p = calc_voxel_points(idx, numVoxels, min_point, voxel_strides, new_scanline);

	voxel.make_tetrahedron(p[0], p[3], p[5], p[1]);
	voxel.make_tetrahedron(p[0], p[3], p[2], p[6]);
	voxel.make_tetrahedron(p[0], p[4], p[5], p[6]);
	voxel.make_tetrahedron(p[5], p[6], p[7], p[3]);
	assert(voxel.is_valid());
}

void calc_voxel_from_idx_hex(unsigned int idx, std::array<unsigned int, 3> numVoxels, ExactPoint min_point,
	const std::array<ExactVector, 3>& voxel_strides, Exact_Polyhedron& voxel)
{
	auto const p = calc_voxel_points(idx, numVoxels, min_point, voxel_strides);

	unsigned int x_idx, y_idx, z_idx;
	convert_voxel_idx_to_coords(idx, numVoxels[0], x_idx, y_idx, z_idx);

	//std::cout << idx << "th voxel(" << x_idx << ", " << y_idx << ", " << z_idx << ")\n";
	CGAL::make_hexahedron(p[0], p[1], p[3], p[2], p[6], p[4], p[5], p[7], voxel);
	assert(voxel.is_valid());
}

float base_cellsize;
ExactPoint global_min_point;
int data_res[3];
//
//void setupVBO(Mesh mesh){
//	// 1. extract vertex and index list
//	std::vector<float> vertices; // (x, y, z)
//	std::vector<unsigned int> indices;
//
//	std::map<Mesh::Vertex_index, unsigned int> vertexMap;
//    unsigned int indexCounter = 0;
//
//    for (auto v : mesh.vertices()) {
//        ExactPoint p(mesh.point(v));
//        vertices.push_back((float)p.x());
//        vertices.push_back((float)p.y());
//        vertices.push_back((float)p.z());
//        vertexMap[v] = indexCounter++;
//    }
//
//    for (auto f : mesh.faces()) {
//        std::vector<unsigned int> faceIndices;
//        for (auto v : CGAL::vertices_around_face(mesh.halfedge(f), mesh)) {
//            faceIndices.push_back(vertexMap[v]);
//        }
//        if (faceIndices.size() == 3) { // 삼각형일 경우
//            indices.insert(indices.end(), faceIndices.begin(), faceIndices.end());
//        }
//    }
//
//}


std::vector<bool> voxelize(Mesh surface, Exact_Polyhedron poly)
{

	float se_size = 0.1f;
	float margin = se_size + 2.0f / (2 * BASE_RESOLUTION);

	Tree mesh_tree(faces(poly).first, faces(poly).second, poly);
	//Tree mesh_tree(faces(surface).first, faces(surface).second, surface);
	CGAL::Bbox_3 bbox_origin = mesh_tree.bbox();
	float longest_axis = std::max(bbox_origin.xmax() - bbox_origin.xmin(),
		std::max(bbox_origin.ymax() - bbox_origin.ymin(), bbox_origin.zmax() - bbox_origin.zmin()));
	float thickness = longest_axis / BASE_RESOLUTION;
	float offset = 2 * thickness + margin;
	float axis_len = longest_axis + 2 * offset;
	float cell_size = axis_len / BASE_RESOLUTION;

	float new_xmin = bbox_origin.xmin() - offset;
	float new_ymin = bbox_origin.ymin() - offset;
	float new_zmin = bbox_origin.zmin() - offset;

	CGAL::Bbox_3 grid_aabb(
		new_xmin,
		new_ymin,
		new_zmin,
		new_xmin + axis_len,
		new_ymin + axis_len,
		new_zmin + axis_len
	);

	ExactPoint grid_min(grid_aabb.xmin(), grid_aabb.ymin(), grid_aabb.zmin());

	// index where the actual data is finished at each axis
	int data_resolution[3] = {
		static_cast<int>(ceil((bbox_origin.xmax() - grid_aabb.xmin()) / cell_size)),
		static_cast<int>(ceil((bbox_origin.ymax() - grid_aabb.ymin()) / cell_size)),
		static_cast<int>(ceil((bbox_origin.zmax() - grid_aabb.zmin()) / cell_size))
	};
	data_res[0] =
		static_cast<int>(ceil((bbox_origin.xmax() - grid_aabb.xmin()) / cell_size)),
		data_res[1] =
		static_cast<int>(ceil((bbox_origin.ymax() - grid_aabb.ymin()) / cell_size)),
		data_res[2] =
		static_cast<int>(ceil((bbox_origin.zmax() - grid_aabb.zmin()) / cell_size));
	std::cout << "data resolution " << data_resolution[0] << ", " << data_resolution[1] << ", " << data_resolution[2] << "\n";

	std::array<unsigned int, 3> numVoxels = { BASE_RESOLUTION, BASE_RESOLUTION, BASE_RESOLUTION };
	std::array<ExactVector, 3> voxel_strides = { ExactVector(cell_size, 0, 0),
		ExactVector(0, cell_size, 0), ExactVector(0, 0, cell_size) };

	std::vector<unsigned int> intersecting_voxels;

	auto numVoxel = pow(BASE_RESOLUTION, 3);// numVoxels[0] * numVoxels[1] * numVoxels[2];
	bool interior = false;
	unsigned int last_voxel = 0;
	std::vector<bool> voxels_marking(numVoxel, false); // either outside 0, surface 1 or interior 2

	Tree tree(faces(poly).first, faces(poly).second, poly);
	Point_inside inside_tester(tree);

	//Exact_Polyhedron voxels;
	//std::cout << "Check for intersection\n";

#pragma omp parallel for collapse(2) schedule(dynamic)
	for (int i = 0; i < data_resolution[0]; i++) {
		for (int j = 0; j < data_resolution[1]; j++) {
			//if (j % 30 == 0) printf("voxelizing (%d, %d)\n", i, j);
#pragma omp parallel for schedule(dynamic)
			for (int k = 0; k < data_resolution[2]; k++) {

				unsigned int idx = i * pow(BASE_RESOLUTION, 2) + j * BASE_RESOLUTION + k;

				Exact_Polyhedron voxel = Exact_Polyhedron();
				bool new_scanline;

				calc_voxel_from_idx_tets(idx, numVoxels, grid_min, voxel_strides, voxel, &new_scanline);

				if (CGAL::Polygon_mesh_processing::do_intersect(voxel, poly))
				{
#pragma omp critical
					{
						intersecting_voxels.push_back(idx);
						voxels_marking[idx] = true;
					}
					continue;
				}

				bool inside = true;

				for (auto vert : voxel.vertex_handles())
				{
					if (inside_tester(vert->point()) != CGAL::ON_BOUNDED_SIDE)
					{
						inside = false;
					}
				}

				if (inside)
				{
#pragma omp critical
					{
						intersecting_voxels.push_back(idx);
						voxels_marking[idx] = true;
					}
				}
			}
		}
	}


	global_min_point = grid_min;
	base_cellsize = cell_size;

	return voxels_marking;
}


bool check_8cube(int x, int y, int z, VOXEL_GRID prev_grid, int prev_resol) {
	bool result = false;
	for (int x_offset = 0; x_offset < 2; x_offset++) {
		for (int y_offset = 0; y_offset < 2; y_offset++) {
			for (int z_offset = 0; z_offset < 2; z_offset++) {
				int flat_idx = (2 * x + x_offset) * prev_resol * prev_resol +
					(2 * y + y_offset) * prev_resol + 2 * z + z_offset;
				result = (result || prev_grid[flat_idx]);
			}
		}
	}
	if (result) {
		int a = 3;
	}
	return result;
}
std::vector<Node> find_subcells(Node parent, MIPMAP_TYPE mipmap) {

	std::vector<Node> subcells;
	unsigned int parent_x, parent_y, parent_z;
	int parent_resol = BASE_RESOLUTION / pow(2, parent.level);

	convert_voxel_idx_to_coords(parent.pos, parent_resol, parent_x, parent_y, parent_z);

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++) {
				unsigned int idx =
					coords_to_voxel_idx(
						2 * parent_x + i,
						2 * parent_y + j,
						2 * parent_z + k,
						parent_resol * 2
					);

				subcells.push_back({ parent.level - 1, idx });
			}
	return subcells;
}

MIPMAP_TYPE generate_mipmap(VOXEL_GRID grid) {
	MIPMAP_TYPE mm_pyramid;
	mm_pyramid.push_back(grid);

	int total_depth = log2(BASE_RESOLUTION) + 1;

	for (int depth = 1; depth < total_depth; depth++) {
		int resol = BASE_RESOLUTION / pow(2, depth);
		VOXEL_GRID mipmap(pow(resol, 3), false);

		for (int i = 0; i < resol; i++) {
			for (int j = 0; j < resol; j++) {
				for (int k = 0; k < resol; k++) {
					int flat_idx = i * resol * resol + j * resol + k;
					mipmap[flat_idx] = check_8cube(i, j, k, mm_pyramid[depth - 1], resol * 2);
				}
			}
		}


		mm_pyramid.push_back(mipmap);
	}
	return mm_pyramid;
}

typedef struct {
	ExactPoint center;
	float radius;
	CGAL::Bbox_3 bbox;
	bool sphere;

}SE;

typedef struct {
	float scale;
	bool occupied;
} CONTOUR_ELEMENT;

CGAL::Bbox_3 calc_voxel_bbox(unsigned int idx, int resol, ExactPoint min_point,
	float cell_size)
{
	unsigned int x_idx, y_idx, z_idx;

	convert_voxel_idx_to_coords(idx, resol, x_idx, y_idx, z_idx);
	float x_min = CGAL::to_double(min_point.x()) + x_idx * cell_size;
	float y_min = CGAL::to_double(min_point.y()) + y_idx * cell_size;
	float z_min = CGAL::to_double(min_point.z()) + z_idx * cell_size;
	return CGAL::Bbox_3(
		x_min,
		y_min,
		z_min,
		x_min + cell_size,
		y_min + cell_size,
		z_min + cell_size
	);
}

void define_se(Node cell, float radius, SE& se, bool sphere) {

	CGAL::Bbox_3 voxel_bbox = calc_voxel_bbox(cell.pos, BASE_RESOLUTION / pow(2, cell.level), global_min_point, base_cellsize * pow(2, cell.level));
	se.center = ExactPoint(
		(voxel_bbox.xmax() + voxel_bbox.xmin()) / 2.0,
		(voxel_bbox.ymax() + voxel_bbox.ymin()) / 2.0,
		(voxel_bbox.zmax() + voxel_bbox.zmin()) / 2.0
	);
	se.radius = radius;
	float xmin = CGAL::to_double(se.center.x()) - radius;
	float ymin = CGAL::to_double(se.center.y()) - radius;
	float zmin = CGAL::to_double(se.center.z()) - radius;
	se.bbox = CGAL::Bbox_3(
		xmin,
		ymin,
		zmin,
		xmin + radius * 2,
		ymin + radius * 2,
		zmin + radius * 2
	);
	se.sphere = sphere;
	return;
}

float get_shortest_dist(ExactPoint point, CGAL::Bbox_3 cell) {
	double sphere_x = CGAL::to_double(point.x());
	double sphere_y = CGAL::to_double(point.y());
	double sphere_z = CGAL::to_double(point.z());
	float closest_x = std::max(cell.xmin(), std::min(sphere_x, cell.xmax()));
	float closest_y = std::max(cell.ymin(), std::min(sphere_y, cell.ymax()));
	float closest_z = std::max(cell.zmin(), std::min(sphere_z, cell.zmax()));

	// Calculate the distance from the sphere's center to the closest point on the bounding box
	float dist = pow(sphere_x - closest_x, 2) + pow(sphere_y - closest_y, 2) + pow(sphere_z - closest_z, 2);
	dist = std::sqrt(dist);

	return dist;

}

bool does_overlap(Node cell, SE& se) {
	int resol = BASE_RESOLUTION / pow(2, cell.level);
	float cell_size = base_cellsize * pow(2, cell.level);
	CGAL::Bbox_3 cell_bbox = calc_voxel_bbox(cell.pos, resol, global_min_point, cell_size);

	if (se.bbox.xmax() >= cell_bbox.xmin() &&
		se.bbox.ymax() >= cell_bbox.ymin() &&
		se.bbox.zmax() >= cell_bbox.zmin() &&
		se.bbox.xmin() <= cell_bbox.xmax() &&
		se.bbox.ymin() <= cell_bbox.ymax() &&
		se.bbox.zmin() <= cell_bbox.zmax()
		) {
		if (se.sphere) {
			float shortest_dist = get_shortest_dist(se.center, cell_bbox);
			if (shortest_dist > se.radius) {
				return false;
			}
			else {
				return true;
			}
		}
		else return true;
	}
	else return false;
}


VOXEL_GRID executeDilation(MIPMAP_TYPE mipmap) {

	VOXEL_GRID d_grid = mipmap[0];

	int mipmap_depth = mipmap.size();


	std::array<unsigned int, 3> num_voxels = { BASE_RESOLUTION, BASE_RESOLUTION, BASE_RESOLUTION };
#pragma omp parallel for collapse(3) schedule(dynamic)
	for (int x = 0; x < BASE_RESOLUTION; x++) {
		for (int y = 0; y < BASE_RESOLUTION; y++) {
			for (int z = 0; z < BASE_RESOLUTION; z++) {
				std::stack<Node> node_stack;
				unsigned int flat_idx = coords_to_voxel_idx(x, y, z, BASE_RESOLUTION);
				if (d_grid[flat_idx] == true) continue;
				SE se;
				Node current_point = { 0, flat_idx };
				if (BASE_RESOLUTION == 32) {
					define_se(current_point, SE_SIZE * base_cellsize, se, false);
				}
				else if (BASE_RESOLUTION == 64) {
					define_se(current_point, SE_SIZE * base_cellsize, se, false);
				}
				else if (BASE_RESOLUTION == 128) {
					define_se(current_point, SE_SIZE * base_cellsize, se, false);
				}
					
					//define_se(current_point, ((float)BASE_RESOLUTION / 24.0) * base_cellsize, se, false);
				//define_se(current_point, base_cellsize * 0.6, se, true);
				node_stack.push({ mipmap_depth - 1, 0 });
				while (!node_stack.empty() && d_grid[flat_idx] == false) {
					auto top_node = node_stack.top();
					node_stack.pop();

					if (top_node.level == 0) {
						d_grid[flat_idx] = true;
						//d_grid[top_node.pos] = true;
						std::stack<Node> empty_stack;
						node_stack.swap(empty_stack);
						break;
					}
					else {
						std::vector<Node> subcells = find_subcells(top_node, mipmap);
						for (auto& subcell : subcells) {
							bool subcell_val = mipmap[subcell.level][subcell.pos];
							bool overlap = does_overlap(subcell, se);
							if (subcell_val == true && does_overlap(subcell, se)) {
								node_stack.push(subcell);
							}
						}
					}
				}
			}
		}
	}
	return d_grid;
}

MIPMAP_TYPE voxelize_and_mipmap(std::string input_path) {

	std::cout << "Loading surface\n";
	Exact_Polyhedron poly;
	if (!PMP::IO::read_polygon_mesh(input_path, poly) || !CGAL::is_triangle_mesh(poly))
	{
		std::cerr << "Invalid input.\n";
		exit(-1);
	}
	Mesh surface;
	if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(input_path, surface) || surface.is_empty())
	{
		std::cerr << "Invalid input file.\n";
		exit(-1);
	}
	int t1 = clock();
	auto voxel_grid = voxelize(surface, poly);

	int t2 = clock();

	std::cout << "start generating mipmap\n";
	MIPMAP_TYPE mipmap_pyramid = generate_mipmap(voxel_grid);
	int t3 = clock();
	std::cout << "mipmap done. start dilation\n";

	printf("Voxelize elapsed time: %5.3f sec\n", float(t2 - t1) / CLOCKS_PER_SEC);
	printf("Generate Mipmap elapsed time: %5.3f sec\n", float(t3 - t2) / CLOCKS_PER_SEC);
	printf("Total elapsed time (from voxelization to mipmap): %5.3f sec\n", float(t3 - t1) / CLOCKS_PER_SEC);
	return mipmap_pyramid;
}

bool check_neighbor(VOXEL_GRID& d_grid, char axis, int center_x, int center_y, int center_z) {
	int radius = 1;

	unsigned int center_idx = coords_to_voxel_idx(center_x, center_y, center_z, BASE_RESOLUTION);
	bool center_val = d_grid[center_idx];
	bool result = false;

	// check for z-1, z+1
	for (int offset = -radius; offset <= radius; offset++) {

		if (offset == 0) continue;

		int x = center_x, y = center_y, z = center_z;

		if (axis == 'x') x += offset;
		else if (axis == 'y') y += offset;
		else if (axis == 'z') z += offset;

		if (!(x >= 0 && x < BASE_RESOLUTION &&
			y >= 0 && y < BASE_RESOLUTION &&
			z >= 0 && z < BASE_RESOLUTION))
			continue;

		unsigned int neighbor_idx = coords_to_voxel_idx(x, y, z, BASE_RESOLUTION);
		result = result || (d_grid[neighbor_idx] != center_val);
	}

	return result;
}

VOXEL_GRID extract_contour(VOXEL_GRID& d_grid) {

	VOXEL_GRID contour(d_grid.size(), false);


	for (int x = 0; x < BASE_RESOLUTION; x++) {
		for (int y = 0; y < BASE_RESOLUTION; y++) {
			for (int z = 0; z < BASE_RESOLUTION; z++) {
				unsigned int center_idx = coords_to_voxel_idx(x, y, z, BASE_RESOLUTION);
				bool center_val = d_grid[center_idx];
				bool result =
					check_neighbor(d_grid, 'z', x, y, z) ||
					check_neighbor(d_grid, 'y', x, y, z) ||
					check_neighbor(d_grid, 'x', x, y, z);

				contour[center_idx] = !center_val && result;
			}
		}
	}

	return contour;
}

float get_shortest_dist(CGAL::Bbox_3& a, CGAL::Bbox_3& b) {
	// Start by calculating the distance on each axis (x, y, and z)
	float distX = 0.0f;
	if (a.xmax() < b.xmin()) {
		distX = b.xmin() - a.xmax();  // AABB a is to the left of AABB b
	}
	else if (a.xmin() > b.xmax()) {
		distX = a.xmin() - b.xmax();  // AABB a is to the right of AABB b
	}

	float distY = 0.0f;
	if (a.ymax() < b.ymin()) {
		distY = b.ymin() - a.ymax();  // AABB a is below AABB b
	}
	else if (a.ymin() > b.ymax()) {
		distY = a.ymin() - b.ymax();  // AABB a is above AABB b
	}

	float distZ = 0.0f;
	if (a.zmax() < b.zmin()) {
		distZ = b.zmin() - a.zmax();  // AABB a is in front of AABB b
	}
	else if (a.zmin() > b.zmax()) {
		distZ = a.zmin() - b.zmax();  // AABB a is behind AABB b
	}

	// The total shortest distance is the Euclidean distance between the gaps on each axis
	return std::sqrt(distX * distX + distY * distY + distZ * distZ);
}

bool does_overlap_erode(Node cell, CGAL::Bbox_3 p_bbox) {
	int resol = BASE_RESOLUTION / pow(2, cell.level);
	float cell_size = base_cellsize * pow(2, cell.level);

	float scale = 0.6f;
	float base_radius = base_cellsize;
	float radius = base_radius * scale;

	CGAL::Bbox_3 cell_bbox = calc_voxel_bbox(cell.pos, resol, global_min_point, cell_size);
	CGAL::Bbox_3 cell_bbox_pad(
		cell_bbox.xmin() - radius,
		cell_bbox.ymin() - radius,
		cell_bbox.zmin() - radius,
		cell_bbox.xmax() + radius,
		cell_bbox.ymax() + radius,
		cell_bbox.zmax() + radius
	);

	if (p_bbox.xmax() >= cell_bbox_pad.xmin() &&
		p_bbox.ymax() >= cell_bbox_pad.ymin() &&
		p_bbox.zmax() >= cell_bbox_pad.zmin() &&
		p_bbox.xmin() <= cell_bbox_pad.xmax() &&
		p_bbox.ymin() <= cell_bbox_pad.ymax() &&
		p_bbox.zmin() <= cell_bbox_pad.zmax()
		) {

		float shortest_dist = get_shortest_dist(p_bbox, cell_bbox);

		if (shortest_dist > radius) {
			return false;
		}
		else {
			return true;
		}
	}
	return false;
}

void execute_erosion(MIPMAP_TYPE& contour_mipmap, VOXEL_GRID& e_grid, VOXEL_GRID& voxel_grid) {

	int mipmap_depth = contour_mipmap.size();

	
#pragma omp parallel for collapse(3) schedule(dynamic)
	for (int x = 0; x < BASE_RESOLUTION; x++) {
		//if (x % 5 == 0) std::cout << "erosion x : " << x << "\n";
		for (int y = 0; y < BASE_RESOLUTION; y++) {
			for (int z = 0; z < BASE_RESOLUTION; z++) {
				int flat_idx = coords_to_voxel_idx(x, y, z, BASE_RESOLUTION);
				if (e_grid[flat_idx] == false || voxel_grid[flat_idx] == true) continue;
				CGAL::Bbox_3 p_bbox = calc_voxel_bbox(flat_idx, BASE_RESOLUTION, global_min_point, base_cellsize);
				std::stack<Node> node_stack;
				node_stack.push({ mipmap_depth - 1, 0 });
				while (!node_stack.empty() && e_grid[flat_idx] == true) {
					auto top_node = node_stack.top();
					node_stack.pop();

					if (top_node.level == 0) {
						e_grid[flat_idx] = false;
						std::stack<Node> empty_stack;
						node_stack.swap(empty_stack);
						break;
					}
					else {
						std::vector<Node> subcells = find_subcells(top_node, contour_mipmap);
						for (auto& subcell : subcells) {
							bool subcell_val = contour_mipmap[subcell.level][subcell.pos];
							bool overlap = does_overlap_erode(subcell, p_bbox);
							if (subcell_val == true && overlap) {
								node_stack.push(subcell);
							}
						}
					}
				}
			}
		}
	}
}


struct PointHash {
	std::size_t operator()(const ExactPoint& p) const {
		auto h1 = std::hash<double>{}(CGAL::to_double(p.x()));
		auto h2 = std::hash<double>{}(CGAL::to_double(p.y()));
		auto h3 = std::hash<double>{}(CGAL::to_double(p.z()));
		return h1 ^ (h2 << 1) ^ (h3 << 2);
	}
};

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

ExactMesh extract_surface_from_voxels(
	const VOXEL_GRID& grid,
	const std::array<ExactVector, 3>& voxel_strides,
	const ExactPoint& origin
)
{
	const std::array<std::array<int, 3>, 6> neighbors = { {
		{{-1, 0, 0}}, {{1, 0, 0}},  // X 
		{{0, -1, 0}}, {{0, 1, 0}},  // Y 
		{{0, 0, -1}}, {{0, 0, 1}}   // Z
	} };

	ExactMesh output_mesh;
	size_t nx = BASE_RESOLUTION, ny = BASE_RESOLUTION, nz = BASE_RESOLUTION;

	std::unordered_map<ExactPoint, ExactMesh::Vertex_index, PointHash> vertex_map;

	for (int x = 0; x < nx; ++x) {
		for (int y = 0; y < ny; ++y) {
			for (int z = 0; z < nz; ++z) {
				int idx = coords_to_voxel_idx(x, y, z, BASE_RESOLUTION);
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
					int n_idx = coords_to_voxel_idx(nx, ny, nz, BASE_RESOLUTION);
					if (nx >= 0 && nx < BASE_RESOLUTION &&
						ny >= 0 && ny < BASE_RESOLUTION &&
						nz >= 0 && nz < BASE_RESOLUTION &&
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

void decimation(MyMesh& vcg_mesh, const int smoothIterations, const int targetNumFaces) {

	tri::Smooth<MyMesh>::VertexCoordLaplacianHC(vcg_mesh, smoothIterations);

	TriEdgeCollapseQuadricParameter qparams;
	qparams.QualityThr = .3;

	float TargetError = std::numeric_limits<float>::max();
	std::cout << "target error: " << TargetError << "\n";
	TargetError = 0.001f;

	qparams.QualityCheck = true;
	qparams.NormalCheck = true;
	qparams.OptimalPlacement = true;
	qparams.ScaleIndependent = true;
	qparams.PreserveTopology = true;

	bool CleaningFlag = true;
	if (CleaningFlag) {
		int dup = tri::Clean<MyMesh>::RemoveDuplicateVertex(vcg_mesh);
		int unref = tri::Clean<MyMesh>::RemoveUnreferencedVertex(vcg_mesh);

		//printf("Removed %i duplicate and %i unreferenced vertices from mesh \n", dup, unref);
	}
	int FinalSize = 400;
	//printf("reducing it to %i\n", FinalSize);

	vcg::tri::UpdateBounding<MyMesh>::Box(vcg_mesh);

	// decimator initialization
	vcg::LocalOptimization<MyMesh> DeciSession(vcg_mesh, &qparams);

	int t1 = clock();
	DeciSession.Init<MyTriEdgeCollapse>();
	int t2 = clock();

	// printf("BEFORE: mesh  %d %d \n", vcg_mesh.vn, vcg_mesh.fn);
	// printf("Initial Heap Size %i\n", int(DeciSession.h.size()));

	DeciSession.SetTargetSimplices(targetNumFaces);
	DeciSession.SetTimeBudget(0.5f);
	DeciSession.SetTargetOperations(100000);
	//if (TargetError < std::numeric_limits<float>::max()) DeciSession.SetTargetMetric(TargetError);

	//while (DeciSession.DoOptimization() && vcg_mesh.fn > FinalSize && DeciSession.currMetric < TargetError)

	while (DeciSession.DoOptimization() && vcg_mesh.fn > targetNumFaces)
		printf("Current Mesh size %7i heap sz %9i err %9g \n", vcg_mesh.fn, int(DeciSession.h.size()), DeciSession.currMetric);

	int t3 = clock();
	if (CleaningFlag) {
		int dup = tri::Clean<MyMesh>::RemoveDuplicateVertex(vcg_mesh);
		int unref = tri::Clean<MyMesh>::RemoveUnreferencedVertex(vcg_mesh);
		int deg_face = tri::Clean<MyMesh>::RemoveDegenerateFace(vcg_mesh);
		int dup_face = tri::Clean<MyMesh>::RemoveDuplicateFace(vcg_mesh);

		//tri::UpdateNormal<MyMesh>::PerVertexPerFace(vcg_mesh);

		//printf("Removed %i duplicate and %i unreferenced vertices from mesh \n", dup, unref);
		//printf("Removed %i duplicate and %i unreferenced faces from mesh \n", dup_face, deg_face);
	}

	//printf("mesh  %d %d Error %g \n", vcg_mesh.vn, vcg_mesh.fn, DeciSession.currMetric);
	printf("\nCompleted decimation in (%5.3f+%5.3f) sec\n", float(t2 - t1) / CLOCKS_PER_SEC, float(t3 - t2) / CLOCKS_PER_SEC);

	//return vcg_mesh;

}

GLFWwindow* initOpenGL() {

	if (!glfwInit()) {
		std::cerr << "Failed to initialize GLFW" << std::endl;
		return nullptr;
	}

	// Set OpenGL context version (OpenGL 4.5)
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Create window
	GLFWwindow* window = glfwCreateWindow(1, 1, "OpenGL 4.5 - GLSL 450", NULL, NULL);
	if (!window) {
		std::cerr << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return nullptr;
	}

	// Bind OpenGL 
	glfwMakeContextCurrent(window);

	// GLEW 초기화
	glewExperimental = GL_TRUE;
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		std::cerr << "GLEW init failed: " << glewGetErrorString(err) << std::endl;
		return nullptr;
	}

	// 이제 OpenGL 함수 호출 가능!
	std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
	std::cout << "GLSL Version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;

	return window;
}


// TODO: test whether this is needed and remove if not.

void ConvertBucketToUnpackedGrid(const std::vector<unsigned int>& buck_vox_grid,
	int res, int num_bit_vox,
	std::vector<bool>& unpack_vox_grid) {
	// Unpacked grid의 크기는 2x2x2 범위로 하나의 voxel이 정의된 크기만큼 나눔
	int unpack_res_x = res / 2, unpack_res_y = res / 2, unpack_res_z = res / 2;
	unpack_vox_grid.resize(unpack_res_x * unpack_res_y * unpack_res_z, false);

	int res_x = res, res_y = res, res_z = res / ((32 / num_bit_vox));

	// 2x2x2 block to one voxel
	for (int i = 0; i < res_x; i++) {
		for (int j = 0; j < res_y; j++) {
			for (int k = 0; k < res_z; k++) {
				unsigned int value = buck_vox_grid[i + res_x * j + res_x * res_y * k];

				// 2x2x2 영역에 대해 비트 연산 (OR 연산)하여 하나의 큰 voxel을 만듬
				if (value != 0) {

					if (num_bit_vox == 1) {
						// 1 비트씩 처리
						for (int s = 0; s < 32; s++) {
							if ((value & (0x00000001)) == 0x00000001) {
								// 위치 계산 (현재 i, j, k는 각 2x2x2 블록의 시작 위치)
								int pos[3] = { i, j, k * 32 + s };
								int target_pos[3] = { pos[0] / 2, pos[1] / 2, pos[2] / 2 };

								int target_pos_flat = target_pos[2] + target_pos[1] * unpack_res_z + target_pos[0] * unpack_res_z * unpack_res_y;
								unpack_vox_grid[target_pos_flat] = true;
							}
							value /= 2;  // 2^(1 bit shifts)
						}
					}

					if (num_bit_vox == 8) {
						// 8 비트씩 처리
						for (int s = 0; s < 4; s++) {
							if ((value & (0x00000001)) == 0x00000001) {
								// 위치 계산 (현재 i, j, k는 각 2x2x2 블록의 시작 위치)
								int position = (i / 2) + unpack_res_x * (j / 2) + unpack_res_x * unpack_res_y * (k / 2);
								unpack_vox_grid[position] = true;  // Set bit to true
							}
							value /= 256;  // 2^(8 bit shifts)
						}
					}
				}
			}
		}
	}
}

void run_voxlize_shader(GLuint program, bool is_surface, float *bbox_min, float rescale, int res, float sq_ar_thresh, GLuint vao, int num_indices) {
	////////////// SURFACE VOXELIZATION

	std::cout << "Starting Surface Voxelization!!\n";

	GLuint query;
	GLuint64 elapsed_time;
	glGenQueries(1, &query);


	// 1) Use the Volume Voxelization Shader
	glUseProgram(program);
	printf("Using Conservative Surface Voxelization Program");


	// 2) Initialization of the shader parameters
	// Vertex and Fragment Shader Common Setup
	glUniform3f(glGetUniformLocation(program, "bbox_min"),
		bbox_min[0], bbox_min[1], bbox_min[2]);
	glUniform1f(glGetUniformLocation(program, "rescale"), rescale);
	glUniform1ui(glGetUniformLocation(program, "res"), res);
	
	if (is_surface) {
		// Geometry Shader Setup
		glUniform1f(glGetUniformLocation(program, "sq_ar_thresh"), sq_ar_thresh);
	}

	// Fragment Shader Setup : assign image unit
	glUniform1i(glGetUniformLocation(program, "vox_grid"), 0);

	// 3) Single Pass  Voxelization
	
	if (glewIsSupported("GL_NV_conservative_raster")) {
		if (is_surface) {
			glEnable(GL_CONSERVATIVE_RASTERIZATION_NV);
			std::cout << "enabling GL_CONSERVATIVE_RASTERIZATION_NV\n";
		}
		else {
			glDisable(GL_CONSERVATIVE_RASTERIZATION_NV);
			std::cout << "disabling GL_CONSERVATIVE_RASTERIZATION_NV\n";
		}
	}
	else if (glewIsSupported("GL_INTEL_conservative_rasterization")) {
		if (is_surface) {
			glEnable(GL_CONSERVATIVE_RASTERIZATION_INTEL);
			std::cout << "enabling GL_CONSERVATIVE_RASTERIZATION_INTEL\n";
		}
		else {
			glDisable(GL_CONSERVATIVE_RASTERIZATION_INTEL);
			std::cout << "disabling GL_CONSERVATIVE_RASTERIZATION_INTEL\n";
		}
		
	}
	else {
		std::cerr << "[WARNING] GL_NV_conservative_raster/GL_INTEL_conservative_rasterization is not supported on this GPU." << std::endl;
	}

	glBeginQuery(GL_TIME_ELAPSED, query);

	glBindVertexArray(vao);
	glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
	glBindVertexArray(0);

	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	glEndQuery(GL_TIME_ELAPSED);

	// Retrieving the recorded elapsed time : wait until
	// the query result is available
	int done = 0;
	while (!done)
		glGetQueryObjectiv(query, GL_QUERY_RESULT_AVAILABLE, &done);

	// Get the query result for elapsed time
	glGetQueryObjectui64v(query, GL_QUERY_RESULT, &elapsed_time);
	
	if (is_surface) {
		std::cout << "[Voxelizer] : " << "conservative surface voxelization done in "
			<< elapsed_time / 1000000.0 << " ms." << std::endl;
	}
	else {
		std::cout << "[Voxelizer] : " << "conservative volume voxelization done in "
			<< elapsed_time / 1000000.0 << " ms." << std::endl;
	}
}

std::vector<bool> opengl_voxelization(std::string filename) {
	GLFWwindow* window = initOpenGL();

	Eigen::MatrixXd V_model, N_model;
	Eigen::MatrixXi T_model;

	if (!load_mesh(filename, V_model, T_model, 1.0))
	{
		std::vector<bool> dm;
		std::cerr << "Failed to load mesh file\n";
		return dm;
	}

	calcNormals(V_model, T_model, N_model);

	std::vector<float> vertices;
	std::vector<float> normals;
	std::vector<unsigned int> indices;

	float bbox_min[3] = { FLT_MAX, FLT_MAX, FLT_MAX };
	float bbox_max[3] = { FLT_MIN, FLT_MIN, FLT_MIN };

	// Convert Eigen::MatrixXd (vertices) to std::vector<float>
	for (int i = 0; i < V_model.rows(); ++i) {
		for (int j = 0; j < V_model.cols(); ++j) {
			vertices.push_back(static_cast<float>(V_model(i, j)));

			if (V_model(i, j) < bbox_min[j]) bbox_min[j] = static_cast<float>(V_model(i, j));
			else if (V_model(i, j) > bbox_max[j]) bbox_max[j] = static_cast<float>(V_model(i, j));
		}
	}

	for (int i = 0; i < N_model.rows(); ++i) {
		for (int j = 0; j < N_model.cols(); ++j) {
			normals.push_back(static_cast<float>(N_model(i, j)));
		}
	}

	// Convert Eigen::MatrixXi (faces) to std::vector<unsigned int>
	for (int i = 0; i < T_model.rows(); ++i) {
		for (int j = 0; j < T_model.cols(); ++j) {
			indices.push_back(static_cast<unsigned int>(T_model(i, j)));
		}
	}

	// 0) Set the cell size
	float se_size = 0.1f;
	float margin = se_size + 2.0f / (2 * BASE_RESOLUTION);
	float longest_axis = std::max(bbox_max[0] - bbox_min[0],
		std::max(bbox_max[1] - bbox_min[1], bbox_max[2] - bbox_min[2]));
	float thickness = longest_axis / BASE_RESOLUTION;
	float offset = 2 * thickness + margin;
	float axis_len = longest_axis + 2 * offset;
	float cell_size = axis_len / BASE_RESOLUTION;
	base_cellsize = cell_size;

	float new_xmin = bbox_min[0] - offset;
	float new_ymin = bbox_min[1] - offset;
	float new_zmin = bbox_min[2] - offset;

	float grid_bbox_min[3] = {
		new_xmin,
		new_ymin,
		new_zmin,
	};

	float grid_bbox_max[3] = {
		new_xmin + axis_len,
		new_ymin + axis_len,
		new_zmin + axis_len
	};

	ExactPoint grid_min(new_xmin, new_ymin, new_zmin);
	global_min_point = grid_min;

	// 1) init vbo
	GLuint vertex_vbo_id_ = 0;
	GLuint normal_vbo_id_ = 0;
	GLuint index_vbo_id_ = 0;
	bool use_vbo_ = true;

	GLuint vao = 0;

	{
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);

		// Create vertex vbo and write data
		glGenBuffers(1, &vertex_vbo_id_);
		glBindBuffer(GL_ARRAY_BUFFER, vertex_vbo_id_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), vertices.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
		glEnableVertexAttribArray(0);

		// Create normal vbo and write data
		glGenBuffers(1, &normal_vbo_id_);
		glBindBuffer(GL_ARRAY_BUFFER, normal_vbo_id_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * normals.size(), normals.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
		glEnableVertexAttribArray(1);

		// Create EBO (Index Buffer Object) and write data
		glGenBuffers(1, &index_vbo_id_);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_vbo_id_);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

		// Unbind VAO
		glBindVertexArray(0);
	}


	int base_res_ = BASE_RESOLUTION;
	int res = 2 * base_res_; // the base_res_ is the resolution of the packed layout
	int num_bit_vox = 1; // number of bits in one bucket

	// Load shader programs
	GLuint program_volume = genVGFProgram(
		"assets/shaders/voxelizer/vol_vox.vert",
		"assets/shaders/voxelizer/vol_vox.geo",
		"assets/shaders/voxelizer/vol_vox_bit.frag");

	GLuint program_surface = genVGFProgram(
		"assets/shaders/voxelizer/surf_vox_conserv.vert",
		"assets/shaders/voxelizer/surf_vox_conserv.geo",
		"assets/shaders/voxelizer/surf_vox_conserv_bit.frag");

	// func: buildFBO
	GLuint vox_fbo_;
	GLuint vox_fbo_tex_;
	GLuint vox_tex_;
	int bucket_size = 32;
	//int num_bit_vox = 1;
	int res_x = res, res_y = res, res_z = res / ((bucket_size / num_bit_vox));
	// Some general OpenGL setup :	
	// Render Target (FBO + attached texture), viewport, depth test etc.
	glGenFramebuffers(1, &vox_fbo_);
	glBindFramebuffer(GL_FRAMEBUFFER, vox_fbo_);
	//("Binding to a dummy fbo");

	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, &vox_fbo_tex_);
	glBindTexture(GL_TEXTURE_2D, vox_fbo_tex_);
	//("Binding dummy texture");

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	printf("Setting dummy texture Tex Parameters");

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, res_x, res_y, 0,
		GL_RGBA, GL_FLOAT, NULL);
	printf("Allocating dummy texture");

	glFramebufferTexture2D(GL_FRAMEBUFFER,
		GL_COLOR_ATTACHMENT0,
		GL_TEXTURE_2D,
		vox_fbo_tex_, 0);
	printf("Attaching dummy texture to dummy fbo");

	glViewport(0, 0, res_x, res_y);
	printf("Setting viewport");

	glDisable(GL_DEPTH_TEST);
	//printOpenGLError ("Disabling Depth Test");

	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	//printOpenGLError ("Disabling framebuffer writing");

	// Necessary with the non-conservative rasterizer
	glDisable(GL_CULL_FACE);
	printf("Disabling Face Culling");

	// Create the texture we're going to render to
	glEnable(GL_TEXTURE_3D);
	glGenTextures(1, &vox_tex_);
	printf("Generating voxel texture");

	// "Bind" the newly created texture : all future texture
	// functions will modify this texture
	glBindTexture(GL_TEXTURE_3D, vox_tex_);
	printf("Binding voxel texture");

	// Give an empty image to OpenGL ( the last "0" )
	int tex_level = 0;
	int border = 0;
	glTexImage3D(GL_TEXTURE_3D, tex_level, GL_R32UI, res_x, res_y, res_z,
		border, GL_RED_INTEGER, GL_UNSIGNED_INT, 0);
	printf("Allocating voxel texture");

	// The two following line are absolutely needed !!!!!
	// But I don't know exactly why :-( ....
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	printf("Setting magic Tex Parameter");

	// Clear the texture
	unsigned int vox_tex_clear_value = 0;
	glBindTexture(GL_TEXTURE_3D, vox_tex_);
	printf("Binding voxel texture");
	glClearTexImage(vox_tex_, tex_level, GL_RED_INTEGER, GL_UNSIGNED_INT,
		&vox_tex_clear_value);
	printf("Clearing voxel texture");

	// Not sure if this barrier is necessary
	glMemoryBarrier(GL_TEXTURE_UPDATE_BARRIER_BIT);
	printf("Querying Texture Update Barrier");

	// Bind the texture to an image unit
	glBindImageTexture(0, vox_tex_, tex_level,
		GL_TRUE,
		0,
		GL_READ_WRITE,
		GL_R32UI
	);
	printf("Binding voxel texture to image unit");

	// func: voxelizevolume

	float rescale = 2.0 / axis_len;
	// Timing related query
	run_voxlize_shader(program_volume, false, grid_bbox_min, rescale, res, 0, vao, indices.size());
	run_voxlize_shader(program_surface, true, grid_bbox_min, rescale, res, 50.f * 50.f, vao, indices.size());

	
	// function clearFBO()
	// 4) Delete the dummy fbo and its attached texture
	glDeleteTextures(1, &vox_fbo_tex_);
	printf("Deleting the dummy texture");
	glDeleteFramebuffers(1, &vox_fbo_);
	printf("Deleting the dummy fbo");

	// 4) Restore a "standard" OpenGL state
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	printf("Enabling Back-buffer writing");

	glEnable(GL_CULL_FACE);
	printf("Enabling Face Culling");

	glDisable(GL_TEXTURE_3D);
	printf("Disabling 3D Textures");

	glEnable(GL_DEPTH_TEST);
	printf("Enabling Depth Test");

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	printf("Binding to backbuffer");

	std::vector<unsigned int> buck_vox_grid(res_x * res_y * res_z, 0);
	glBindTexture(GL_TEXTURE_3D, vox_tex_);
	printf("Binding voxel texture");
	int get_tex_level = 0;
	glGetTexImage(GL_TEXTURE_3D, get_tex_level,
		GL_RED_INTEGER,
		GL_UNSIGNED_INT,
		&buck_vox_grid[0]);
	printf("Retrieving Data");

	// Delete VBO and VAO
	glDeleteBuffers(1, &vertex_vbo_id_);
	glDeleteBuffers(1, &normal_vbo_id_);
	glDeleteBuffers(1, &index_vbo_id_);
	glDeleteVertexArrays(1, &vao);

	std::vector<bool> result_grid;
	ConvertBucketToUnpackedGrid(buck_vox_grid, res, num_bit_vox, result_grid);
	glfwDestroyWindow(window);
	glfwTerminate();
	return result_grid;

}

void GenerateCageFromMeshOperation::Execute() {
	//std::cout << "Enter resolution: ";
	//std::cin >> BASE_RESOLUTION;

	//std::cout << "Enter SE_SIZE: ";
	//std::cin >> SE_SIZE;
	int cage_start = clock();
	
	std::string filename = _params._meshfilepath.string();
	std::string outputfilename = _params._cagefilepath.string();

	//extract input model name
	std::string obj = filename.substr(filename.find_last_of('/') + 1, filename.find_last_of('.') - 1);
	std::string filepath = filename.substr(0, filename.find_last_of('/') + 1);
	std::string intermediate_path = filepath + obj + "_interm.obj";
	std::string voxel_result_path = filepath + obj + "_voxel.off";

	VOXEL_GRID& e_grid = _params._closingResult;

	std::cout << "Generating Cage for " << obj << std::endl;
	// generate voxel grid and mipmap
	if (e_grid.empty())
	{	
		//MIPMAP_TYPE mipmap = voxelize_and_mipmap(filename);
		int clock_voxel_start = clock();
		std::vector<bool> voxel_result = opengl_voxelization(filename);
		int clock_voxel_end = clock();
		printf("[Voxelization] elapsed time: %5.3f sec\n", float(clock_voxel_end - clock_voxel_start) / CLOCKS_PER_SEC);

		/*std::array<ExactVector, 3> voxel_strides_v = { ExactVector(base_cellsize, 0, 0),
		ExactVector(0, base_cellsize, 0), ExactVector(0, 0, base_cellsize) };
		ExactMesh extracted_surface_v = extract_surface_from_voxels(voxel_result, voxel_strides_v, global_min_point);
		CGAL::write_off(voxel_result_path.c_str(), extracted_surface_v);*/
		int mipmap_start = clock();
		MIPMAP_TYPE mipmap = generate_mipmap(voxel_result); std::cout << "mipmap done\n";
		int mipmap_end = clock();
		printf("[Mipmap] elapsed time: %5.3f sec\n", float(mipmap_end - mipmap_start) / CLOCKS_PER_SEC);

		// dilation
		int dil_start = clock();
		VOXEL_GRID d_grid = executeDilation(mipmap); std::cout << "dilation done\n";
		int dil_end = clock();
		printf("[Dilation] elapsed time: %5.3f sec\n", float(dil_end - dil_start) / CLOCKS_PER_SEC);


		// extract contour and generate mipmap of the contour
		//std::cout << "drawing done. start contour extraction\n";
		int con_start = clock();
		VOXEL_GRID contour = extract_contour(d_grid); std::cout << "contour done\n";
		int con_end = clock();
		printf("[Contour] elapsed time: %5.3f sec\n", float(con_end - con_start) / CLOCKS_PER_SEC);

		//std::cout << "contour extraction done\n";
		MIPMAP_TYPE contour_pyramid = generate_mipmap(contour);

		// erosion
		e_grid.resize(d_grid.size());
		e_grid.assign(d_grid.begin(), d_grid.end());
		int erose_start = clock();
		execute_erosion(contour_pyramid, e_grid, mipmap[0]); std::cout << "erosion done\n";
		int erose_end = clock();
		printf("[Erosion] elapsed time: %5.3f sec\n", float(erose_end - erose_start) / CLOCKS_PER_SEC);

	}
	// Extract the surface from the closed grid
	std::array<ExactVector, 3> voxel_strides = { ExactVector(base_cellsize, 0, 0),
	ExactVector(0, base_cellsize, 0), ExactVector(0, 0, base_cellsize) };
	ExactMesh extracted_surface = extract_surface_from_voxels(e_grid, voxel_strides, global_min_point);
	CGAL::write_off(intermediate_path.c_str(), extracted_surface); std::cout << "extraction done\n";

	// Simplification
	MyMesh final_mesh;
	tri::io::ImporterOFF<MyMesh>::Open(final_mesh, intermediate_path.c_str());
	decimation(final_mesh, _params._smoothIterations, _params._targetNumFaces);
	std::string output_path = filepath + obj + "_cage.obj";

	tri::io::ExporterOBJ<MyMesh>::Save(final_mesh, outputfilename.c_str(), tri::io::Mask::IOM_BITPOLYGONAL);

	int cage_end = clock();
	printf("[Cage Generation] elapsed time: %5.3f sec\n", float(cage_end - cage_start) / CLOCKS_PER_SEC);
}