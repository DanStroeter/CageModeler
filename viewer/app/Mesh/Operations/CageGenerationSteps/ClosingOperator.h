#pragma once

#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <omp.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <Mesh/Operations/CageGenerationSteps/Utils.h>

//#include <CGAL/Polygon_mesh_processing/intersection.h>
//#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
//#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
//#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
//#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
//#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
//#include <CGAL/Polygon_mesh_processing/compute_normal.h>
//#include <CGAL/Polygon_mesh_processing/corefinement.h>
//#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
//#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel			Exact_Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel			Inexact_Kernel;
typedef CGAL::Polyhedron_3<Inexact_Kernel>							Inexact_Polyhedron;
typedef CGAL::Polyhedron_3<Exact_Kernel>							Exact_Polyhedron;
typedef Exact_Kernel::Point_3										ExactPoint;
typedef Inexact_Kernel::Point_3										InexactPoint;
typedef Exact_Kernel::Vector_3										ExactVector;
typedef CGAL::Surface_mesh<InexactPoint>							Mesh;
//typedef CGAL::AABB_face_graph_triangle_primitive<Exact_Polyhedron>	Primitive;

//typedef CGAL::AABB_traits<Exact_Kernel, Primitive>					Traits;
//typedef CGAL::AABB_tree<Traits>										Tree;
//typedef CGAL::Side_of_triangle_mesh<Exact_Polyhedron, Exact_Kernel>	Point_inside;
typedef std::vector<bool> VOXEL_GRID;
typedef std::vector<VOXEL_GRID>	MIPMAP_TYPE;

namespace PMP = CGAL::Polygon_mesh_processing;


class ClosingOperator : public Utils {

public:
	ClosingOperator(int resolution, float seScale, float cellSize, std::array<float, 3> bboxMin, VOXEL_GRID& voxelGrid) :
		_resolution(resolution),
		_seScale(seScale),
		_cellSize(cellSize),
		_bboxMin(bboxMin),
		_voxelGrid(voxelGrid)
	{
		GenerateMipmap(_voxelGrid, _mipmapPyramid);
	}

private:
	float _seScale;
	float _cellSize;
	int _resolution;
	std::array<float, 3> _bboxMin;
	VOXEL_GRID _voxelGrid;
	VOXEL_GRID _dGrid; // Dilated Voxel Grid
	VOXEL_GRID _cGrid; // Extracted Contour Grid
	VOXEL_GRID _eGrid; // Erosed Voxel Grid
	MIPMAP_TYPE _mipmapPyramid;
	MIPMAP_TYPE _contourPyramid;

	typedef struct {
		ExactPoint center;
		float radius;
		CGAL::Bbox_3 bbox;
		bool sphere;
	}SE;
	typedef struct {
		int level;
		unsigned int pos;
	} Node;

	bool check_8cube(int x, int y, int z, VOXEL_GRID prev_grid, int prev_resol) {
		bool result = false;
		for (int x_offset = 0; x_offset < 2; x_offset++) {
			for (int y_offset = 0; y_offset < 2; y_offset++) {
				for (int z_offset = 0; z_offset < 2; z_offset++) {
					int flat_idx = coords_to_voxel_idx(2 * x + x_offset, 2 * y + y_offset, 2 * z + z_offset, prev_resol);
					result = (result || prev_grid[flat_idx]);
				}
			}
		}
		return result;
	}
	CGAL::Bbox_3 calc_voxel_bbox(unsigned int idx, int resol, float cell_size)
	{
		unsigned int x_idx, y_idx, z_idx;
		ExactPoint min_point(_bboxMin[0], _bboxMin[1], _bboxMin[2]);
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
	std::vector<Node> find_subcells(Node parent, MIPMAP_TYPE mipmap) {

		std::vector<Node> subcells;
		unsigned int parent_x, parent_y, parent_z;
		int parent_resol = _resolution / pow(2, parent.level);

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
	void define_se(Node cell, float radius, SE& se, bool sphere) {

		CGAL::Bbox_3 voxel_bbox = calc_voxel_bbox(cell.pos, _resolution / pow(2, cell.level), _cellSize * pow(2, cell.level));
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
	bool does_overlap(Node cell, SE& se) {
		int resol = _resolution / pow(2, cell.level);
		float cell_size = _cellSize * pow(2, cell.level);
		CGAL::Bbox_3 cell_bbox = calc_voxel_bbox(cell.pos, resol, cell_size);

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
	bool does_overlap_erode(Node cell, CGAL::Bbox_3 p_bbox) {

		int resol = _resolution / pow(2, cell.level);
		float cell_size = _cellSize * pow(2, cell.level);

		float erode_scale = 0.4;
		float base_radius = _cellSize;
		float radius = base_radius * _seScale * erode_scale;

		CGAL::Bbox_3 cell_bbox = calc_voxel_bbox(cell.pos, resol, cell_size);
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
	bool check_neighbor(VOXEL_GRID& _dGrid, char axis, int center_x, int center_y, int center_z) {
		int radius = 1;

		unsigned int center_idx = coords_to_voxel_idx(center_x, center_y, center_z, _resolution);
		bool center_val = _dGrid[center_idx];
		bool result = false;

		// check for z-1, z+1
		for (int offset = -radius; offset <= radius; offset++) {

			if (offset == 0) continue;

			int x = center_x, y = center_y, z = center_z;

			if (axis == 'x') x += offset;
			else if (axis == 'y') y += offset;
			else if (axis == 'z') z += offset;

			if (!(x >= 0 && x < _resolution &&
				y >= 0 && y < _resolution &&
				z >= 0 && z < _resolution))
				continue;

			unsigned int neighbor_idx = coords_to_voxel_idx(x, y, z, _resolution);
			result = result || (_dGrid[neighbor_idx] != center_val);
		}

		return result;
	}
	void GenerateMipmap(VOXEL_GRID& source, MIPMAP_TYPE& target) {

		target.clear();

		target.push_back(source);

		int max_depth = log2(_resolution) + 1;

		for (int depth = 1; depth < max_depth; depth++) {
			int resol = _resolution / pow(2, depth);
			VOXEL_GRID mipmap(pow(resol, 3), false);

			for (int i = 0; i < resol; i++) {
				for (int j = 0; j < resol; j++) {
					for (int k = 0; k < resol; k++) {
						int flat_idx = i * resol * resol + j * resol + k;
						mipmap[flat_idx] = check_8cube(i, j, k, target[depth - 1], resol * 2);
					}
				}
			}

			target.push_back(mipmap);
		}
	}

public:
	VOXEL_GRID ExecuteDilation() {

		VOXEL_GRID dGrid = _mipmapPyramid[0];

		int mipmap_depth = _mipmapPyramid.size();

		std::array<unsigned int, 3> num_voxels = { _resolution, _resolution, _resolution };
		//#pragma omp parallel for collapse(3) schedule(dynamic)
		int numVoxels = _resolution * _resolution * _resolution;
#pragma omp parallel for schedule(dynamic)
		for (int flat_idx = 0; flat_idx < numVoxels; flat_idx++) {
			std::stack<Node> node_stack;
			//unsigned int flat_idx = coords_to_voxel_idx(x, y, z, _resolution);
			unsigned int x, y, z;
			convert_voxel_idx_to_coords(flat_idx, _resolution, x, y, z);

			if (dGrid[flat_idx]) continue;
			SE se;
			Node current_point = { 0, flat_idx };
			define_se(current_point, _seScale * _cellSize, se, false);

			node_stack.push({ mipmap_depth - 1, 0 }); // push coarsest 1x1x1 mipmap
			while (!node_stack.empty() && dGrid[flat_idx] == false) {
				auto top_node = node_stack.top();
				node_stack.pop();

				if (top_node.level == 0) {
#pragma omp critical
					dGrid[flat_idx] = true;
					break;
				}
				else {
					std::vector<Node> subcells = find_subcells(top_node, _mipmapPyramid);
					for (auto& subcell : subcells) {
						bool subcell_val = _mipmapPyramid[subcell.level][subcell.pos];
						bool overlap = does_overlap(subcell, se);
						if (subcell_val && overlap) {
							node_stack.push(subcell);
						}
					}
				}
			}
		}
		_dGrid = dGrid;
		return _dGrid;
	}

	MIPMAP_TYPE ExtractContour() {

		VOXEL_GRID contour(_dGrid.size(), false);

		for (int x = 0; x < _resolution; x++) {
			for (int y = 0; y < _resolution; y++) {
				for (int z = 0; z < _resolution; z++) {
					unsigned int center_idx = coords_to_voxel_idx(x, y, z, _resolution);
					bool center_val = _dGrid[center_idx];
					bool result =
						check_neighbor(_dGrid, 'z', x, y, z) ||
						check_neighbor(_dGrid, 'y', x, y, z) ||
						check_neighbor(_dGrid, 'x', x, y, z);

					contour[center_idx] = !center_val && result;
				}
			}
		}
		_cGrid = contour;
		GenerateMipmap(_cGrid, _contourPyramid);
		return _contourPyramid;
	}

	VOXEL_GRID ExecuteErosion() {

		_eGrid.resize(_dGrid.size());
		_eGrid.assign(_dGrid.begin(), _dGrid.end());
		int mipmap_depth = _contourPyramid.size();

#pragma omp parallel for collapse(3) schedule(dynamic)
		for (int x = 0; x < _resolution; x++) {
			for (int y = 0; y < _resolution; y++) {
				for (int z = 0; z < _resolution; z++) {
					int flat_idx = coords_to_voxel_idx(x, y, z, _resolution);
					if (_eGrid[flat_idx] == false || _voxelGrid[flat_idx] == true) continue;
					CGAL::Bbox_3 p_bbox = calc_voxel_bbox(flat_idx, _resolution, _cellSize);
					std::stack<Node> node_stack;
					node_stack.push({ mipmap_depth - 1, 0 });
					while (!node_stack.empty() && _eGrid[flat_idx] == true) {
						auto top_node = node_stack.top();
						node_stack.pop();

						if (top_node.level == 0) {
							_eGrid[flat_idx] = false;
							break;
						}
						else {
							std::vector<Node> subcells = find_subcells(top_node, _contourPyramid);
							for (auto& subcell : subcells) {
								bool subcell_val = _contourPyramid[subcell.level][subcell.pos];
								bool overlap = does_overlap_erode(subcell, p_bbox);
								if (subcell_val && overlap) {
									node_stack.push(subcell);
								}
							}
						}
					}
				}
			}
		}

		return _eGrid;
	}
};