#pragma once

#include <string>
#include <vector>
#include <Eigen/Geometry>

Eigen::Vector3d HSVtoRGB(double H, double S, double V);

/**
 * Writes the mesh with per-vertex colors to an OBJ file, prefixed by the given comment
 * lines. Modified from libigl.
 */
bool writeOBJVertexColors(const std::string& file_name, const Eigen::MatrixXd& V, const Eigen::MatrixXd& V_colors,
	const Eigen::MatrixXi& F, const std::vector<std::string>& header_comments = { });

void write_influence_color_map_OBJ(const std::string& file_name, const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& T, const Eigen::MatrixXd& W, const std::vector<int>& control_vertices_idx, int cage_vertices_offset, bool transposeW);
