#include<Mesh/Operations/MeshGenerateCageOperation.h>

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

#define BASE_RESOLUTION 32

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

	// Exact_Polyhedron voxels;
	//std::cout << "Check for intersection\n";
	for (unsigned int i = 0; i < data_resolution[0]; i++) {
		for (unsigned int j = 0; j < data_resolution[1]; j++) {
			//if (j % 30 == 0) printf("voxelizing (%d, %d)\n", i, j);
			for (unsigned int k = 0; k < data_resolution[2]; k++) {

				unsigned int idx = i * pow(BASE_RESOLUTION, 2) + j * BASE_RESOLUTION + k;

				Exact_Polyhedron voxel = Exact_Polyhedron();
				bool new_scanline;

				calc_voxel_from_idx_tets(idx, numVoxels, grid_min, voxel_strides, voxel, &new_scanline);

				if (CGAL::Polygon_mesh_processing::do_intersect(voxel, poly))
				{
					intersecting_voxels.push_back(idx);
					voxels_marking[idx] = true;
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
					intersecting_voxels.push_back(idx);
					voxels_marking[idx] = true;
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

	std::stack<Node> node_stack;
	std::array<unsigned int, 3> num_voxels = { BASE_RESOLUTION, BASE_RESOLUTION, BASE_RESOLUTION };
	for (int x = 0; x < BASE_RESOLUTION; x++) {
		for (int y = 0; y < BASE_RESOLUTION; y++) {
			for (int z = 0; z < BASE_RESOLUTION; z++) {
				unsigned int flat_idx = coords_to_voxel_idx(x, y, z, BASE_RESOLUTION);
				if (d_grid[flat_idx] == true) continue;
				SE se;
				Node current_point = { 0, flat_idx };
				define_se(current_point, 2 * base_cellsize, se, false);
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

VOXEL_GRID execute_erosion(MIPMAP_TYPE& contour_mipmap, VOXEL_GRID& d_grid, VOXEL_GRID& voxel_grid) {

	VOXEL_GRID e_grid = d_grid;

	int mipmap_depth = contour_mipmap.size();

	std::stack<Node> node_stack;

	for (int x = 0; x < BASE_RESOLUTION; x++) {
		//if (x % 5 == 0) std::cout << "erosion x : " << x << "\n";
		for (int y = 0; y < BASE_RESOLUTION; y++) {
			for (int z = 0; z < BASE_RESOLUTION; z++) {
				int flat_idx = coords_to_voxel_idx(x, y, z, BASE_RESOLUTION);
				if (e_grid[flat_idx] == false || voxel_grid[flat_idx] == true) continue;
				CGAL::Bbox_3 p_bbox = calc_voxel_bbox(flat_idx, BASE_RESOLUTION, global_min_point, base_cellsize);

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
	return e_grid;
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

void decimation(MyMesh& vcg_mesh) {

	tri::Smooth<MyMesh>::VertexCoordLaplacianHC(vcg_mesh, 3);

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

	DeciSession.SetTargetSimplices(FinalSize);
	DeciSession.SetTimeBudget(0.5f);
	DeciSession.SetTargetOperations(100000);
	//if (TargetError < std::numeric_limits<float>::max()) DeciSession.SetTargetMetric(TargetError);

	//while (DeciSession.DoOptimization() && vcg_mesh.fn > FinalSize && DeciSession.currMetric < TargetError)
	while (DeciSession.DoOptimization() && vcg_mesh.fn > FinalSize)
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

void GenerateCageFromMeshOperation::Execute(){

 std::string filename=_params._meshfilepath;
 std::string outputfilename=_params._cagefilepath;

//extract input model name
std::string obj=filename.substr(filename.find_last_of('/')+1,filename.find_last_of('.')-1);
std::string filepath=filename.substr(0,filename.find_last_of('/')+1);
std::string intermediate_path=filepath+obj+"_interm.obj";

 // generate voxel grid and mipmap
	MIPMAP_TYPE mipmap = voxelize_and_mipmap(filename);
	
	// dilation
	VOXEL_GRID d_grid = executeDilation(mipmap);

	// extract contour and generate mipmap of the contour
	//std::cout << "drawing done. start contour extraction\n";
	VOXEL_GRID contour = extract_contour(d_grid);

	//std::cout << "contour extraction done\n";
	MIPMAP_TYPE contour_pyramid = generate_mipmap(contour);

	// erosion
	VOXEL_GRID e_grid = execute_erosion(contour_pyramid, d_grid, mipmap[0]);
	//std::cout << "erosion done, start surface extraction\n";

	// Extract the surface from the closed grid
	std::array<ExactVector, 3> voxel_strides = { ExactVector(base_cellsize, 0, 0),
	ExactVector(0, base_cellsize, 0), ExactVector(0, 0, base_cellsize) };
	ExactMesh extracted_surface = extract_surface_from_voxels(e_grid, voxel_strides, global_min_point);
    CGAL::write_off(intermediate_path.c_str(), extracted_surface);

     // Simplification
	MyMesh final_mesh;
	tri::io::ImporterOFF<MyMesh>::Open(final_mesh, intermediate_path.c_str());
	decimation(final_mesh);
	std::string output_path = filepath + obj + "_cage.obj";
	
	tri::io::ExporterOBJ<MyMesh>::Save(final_mesh,outputfilename.c_str(),tri::io::Mask::IOM_BITPOLYGONAL);
}