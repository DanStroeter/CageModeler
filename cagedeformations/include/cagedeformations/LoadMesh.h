#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

template<typename vec, typename T>
void scale_verts(std::vector<vec>& verts, T fac)
{
	if (verts.empty())
	{
		return;
	}

	vec min_vert = verts[0], max_vert = verts[0];

	for (unsigned int i = 0; i < verts.size(); ++i)
	{
		auto const vert = verts[i];
		for (unsigned k = 0; k < 3u; ++k)
		{
			min_vert[k] = std::min(min_vert[k], vert[k]);
			max_vert[k] = std::max(max_vert[k], vert[k]);
		}
	}

	const vec mid = { (min_vert[0] + max_vert[0]) * .5f, (min_vert[1] + max_vert[1]) * .5f, (min_vert[2] + max_vert[2]) * .5f };

	for (unsigned int i = 0; i < verts.size(); ++i)
	{
		for (unsigned k = 0; k < 3u; ++k)
		{
			verts[i][k] = (verts[i][k] - mid[k]) * fac + mid[k];
		}
	}
}

bool load_mesh(std::string const & file_name, Eigen::MatrixXd& V, Eigen::MatrixXi & T, double scaling_factor);

void load_cage(Eigen::MatrixXd& V, const std::vector<std::vector<int>>& polys, Eigen::VectorXi& P, Eigen::MatrixXi& CF,
	bool triangulate_quads, Eigen::MatrixXd * V_embedding /*= nullptr*/, bool find_offset /*= false*/);

bool load_cage(std::string const& file_name, Eigen::MatrixXd& V, Eigen::VectorXi& P, Eigen::MatrixXi& CF,
	double scaling_factor, bool triangulate_quads, Eigen::MatrixXd* V_embedding = nullptr, bool find_offset = false);
