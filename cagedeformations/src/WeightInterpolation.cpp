#include <cagedeformations/WeightInterpolation.h>

#include <igl/barycentric_coordinates.h>
#include <igl/EPS.h>
#include <igl/lbs_matrix.h>

#include <iostream>
#include <omp.h>

void interpolateWeightsInEmbedding(const Eigen::MatrixXd& V, const Eigen::MatrixXd& W, const Eigen::MatrixXd& V_embedding,
	const Eigen::MatrixXi& T_embedding, int numCageVertices, Eigen::MatrixXd& W_interpolated, Eigen::MatrixXd& M)
{
	W_interpolated.resize(V.rows(), numCageVertices);

#pragma omp parallel for
	for (int i = 0; i < V.rows(); ++i)
	{
		Eigen::RowVector3d vert = V.row(i);
		bool found = false;

		// Find containing tetrahedra
		// TODO - More efficient approach than linear search
		for (int j = 0; j < T_embedding.rows(); ++j)
		{
			const Eigen::Vector4i tet = T_embedding.row(j);
			const Eigen::RowVector3d v_0 = V_embedding.row(tet(0));
			const Eigen::RowVector3d v_1 = V_embedding.row(tet(1));
			const Eigen::RowVector3d v_2 = V_embedding.row(tet(2));
			const Eigen::RowVector3d v_3 = V_embedding.row(tet(3));
			Eigen::RowVector4d barycentrics;
			found = true;

			igl::barycentric_coordinates(vert, v_0, v_1, v_2, v_3, barycentrics);


			for (int k = 0; k < 4u; ++k)
			{
				if (0. - igl::FLOAT_EPS > barycentrics(k) || 1. + igl::FLOAT_EPS < barycentrics(k))
				{
					found = false;
					break;
				}
			}

			if (!found)
			{
				continue;
			}

			W_interpolated.row(i) = W.row(tet(0)) * barycentrics(0) + W.row(tet(1)) * barycentrics(1) + W.row(tet(2)) * barycentrics(2) +
				W.row(tet(3)) * barycentrics(3);
			break;
		}
		assert(found);
		//std::cout << (static_cast<float>(i) / static_cast<float>(V.rows())) * 100.f << "%\n";
	}

	igl::lbs_matrix(V, W_interpolated, M);
}
