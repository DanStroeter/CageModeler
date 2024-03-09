#pragma once

#include <vector>
#include <Eigen/Geometry>
#include <string>

Eigen::Vector3d HSVtoRGB(double H, double S, double V);
void write_influence_color_map_OBJ(const std::string& file_name, const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& T, const Eigen::MatrixXd& W, const std::vector<int>& control_vertices_idx, int cage_vertices_offset, bool transposeW);