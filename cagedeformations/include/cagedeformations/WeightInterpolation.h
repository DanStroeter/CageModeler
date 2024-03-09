#pragma once

#include <Eigen/Geometry>

void interpolateWeightsInEmbedding(const Eigen::MatrixXd & V, const Eigen::MatrixXd & W, const Eigen::MatrixXd & V_embedding,
	const Eigen::MatrixXi & T_embedding, int numCageVertices, Eigen::MatrixXd & W_interpolated, Eigen::MatrixXd & M);
