#pragma once

#include <string>

#include <Eigen/Geometry>

bool load_fbx_file(const std::string & fbxFile, Eigen::MatrixXd & V_model, Eigen::MatrixXi & T_model, Eigen::MatrixXd& V_cage, Eigen::MatrixXi& T_cage);
