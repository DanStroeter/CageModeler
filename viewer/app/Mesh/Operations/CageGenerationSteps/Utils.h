#pragma once

#include <iostream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel			Exact_Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel			Inexact_Kernel;
typedef CGAL::Polyhedron_3<Inexact_Kernel>							Inexact_Polyhedron;
typedef CGAL::Polyhedron_3<Exact_Kernel>							Exact_Polyhedron;
typedef Exact_Kernel::Point_3										ExactPoint;
typedef Inexact_Kernel::Point_3										InexactPoint;
typedef Exact_Kernel::Vector_3										ExactVector;

struct Utils {

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


	void convert_voxel_idx_to_coords(unsigned int idx, unsigned int numVoxels, unsigned int& x_idx, unsigned int& y_idx, unsigned int& z_idx)
	{
		x_idx = idx / (numVoxels * numVoxels);
		auto const w = idx % (numVoxels * numVoxels);
		y_idx = w / numVoxels;
		z_idx = w % numVoxels;
	}

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

	unsigned int coords_to_voxel_idx(unsigned int x_idx, unsigned int y_idx, unsigned int z_idx, int numVoxels)
	{
		return z_idx + y_idx * numVoxels + x_idx * numVoxels * numVoxels;
	}

};

