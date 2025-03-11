#include <Mesh/Operations/MeshComputeWeightsOperation.h>

#include <cagedeformations/GreenCoordinates.h>
#include <cagedeformations/MaximumEntropyCoordinates.h>
#include <cagedeformations/MaximumLikelihoodCoordinates.h>
#include <cagedeformations/WeightInterpolation.h>
#include <igl/boundary_conditions.h>
#include <igl/harmonic.h>
#include <igl/bbw.h>
#include <igl/lbs_matrix.h>
#include <LBC/DataSetup.h>
#include <LBC/LBCSolver.h>

MeshComputeWeightsOperation::ExecutionResult MeshComputeWeightsOperation::Execute()
{
	// Compute BBW weights matrix.
	Eigen::MatrixXd weights;
	Eigen::MatrixXd interpolatedWeights;
	Eigen::MatrixXd psi;
	std::vector<double> psiTri { };
	std::vector<Eigen::Vector4d> psiQuad { };

	LOG_DEBUG("Computing weights.");

	if (_params._deformationType == DeformationType::Harmonic)
	{
		if (!igl::harmonic(_params._embedding->_vertices, _params._embedding->_faces, _params._b, _params._bc, 1, weights))
		{
			LOG_ERROR("Failed to compute harmonic weights!.");

			return ExecutionResult("Failed to compute harmonic weights");
		}
	}
	else if (_params._deformationType == DeformationType::LBC)
	{
		if (_params._weights.has_value())
		{
			weights = std::move(*_params._weights);
		}
		else
		{
			Eigen::MatrixXd samplePoints(_params._embedding->_vertices.cols(), _params._embedding->_vertices.rows());
			for (auto i = 0; i < _params._embedding->_vertices.rows(); ++i)
			{
				samplePoints.col(i) = _params._embedding->_vertices.row(i);
			}

			Eigen::MatrixXi cellVertices(_params._embedding->_faces.cols(), _params._embedding->_faces.rows());
			for (auto i = 0; i < _params._embedding->_faces.rows(); ++i)
			{
				cellVertices.col(i) = _params._embedding->_faces.row(i);
			}

			LBC::IndexVector controlPointIdx(_params._cagePoints.size());
			for (auto i = 0; i < _params._cagePoints.size(); ++i)
			{
				controlPointIdx(i) = i;
			}

			std::vector<LBC::DataSetup::CageBoundaryFacetInfo> boundaryFacetInfo;
			for (auto i = 0; i < _params._cage._faces.rows(); ++i)
			{
				const Eigen::Vector3i triIndices = _params._cage._faces.row(i);
				std::vector<int> boundaryPoints;

				for (auto j = _params._cagePoints.size(); j < _params._bc.rows(); ++j)
				{
					const auto row = _params._bc.row(j);
					bool contains = true;

					for (auto l = 0; l < 3; ++l)
					{
						if (row(triIndices(l)) == 0)
						{
							contains = false;
							break;
						}
					}

					if (contains)
					{
						boundaryPoints.push_back(_params._b(j));
					}
				}

				auto boundaryPointsVec = LBC::IndexVector(boundaryPoints.size());
				for (std::size_t pointIndex = 0; pointIndex < boundaryPoints.size(); ++pointIndex)
				{
					boundaryPointsVec(static_cast<Eigen::Index>(pointIndex)) = boundaryPoints[pointIndex];
				}

				LBC::DataSetup::CageBoundaryFacetInfo info(triIndices, boundaryPointsVec);
				boundaryFacetInfo.push_back(info);
			}

			LBC::DataSetup ds(samplePoints, controlPointIdx, cellVertices, boundaryFacetInfo, _params._LBCScheme);

			LBC::Param param;
			param.max_iterations = _params._numBBWSteps;
			param.relaxation_alpha = 1.65;
			param.convergence_check_frequency = 10;
			param.output_frequency_ratio = 10;
			param.rel_primal_eps = 0.00000000001;
			param.rel_dual_eps = 0.00000000001;
			param.penalty_weight = 100;
			LBC::LBCSolver solver(param, ds);

			LOG_DEBUG("LBC Solver started.");

			solver.solve();

			LOG_DEBUG("Finished computation.");

			weights = ds.get_full_coordinate_values(solver.get_coordinates());
		}
	}
	else if (_params._deformationType == DeformationType::Green)
	{
		calculateGreenCoordinatesFromQMVC(_params._cage._vertices,
			_params._cage._faces,
			_params._normals,
			_params._mesh._vertices,
			weights,
			psi);
	}
	else if (_params._deformationType == DeformationType::QGC)
	{
		calculateGreenCoordinatesTriQuad(_params._cage._vertices,
			_params._cage._faces,
			_params._mesh._vertices,
			weights,
			psiTri,
			psiQuad);
	}
	else if (_params._deformationType == DeformationType::Somigliana)
	{
		_params._somiglianaDeformer->precompute_somig_coords(false);
	}
	else if (_params._deformationType == DeformationType::MVC)
	{
		computeMVC(_params._cage._vertices,
			_params._cage._faces,
			_params._mesh._vertices,
			weights);
	}
	else if (_params._deformationType == DeformationType::QMVC)
	{
		computeMVCTriQuad(_params._cage._vertices,
			_params._cage._faces,
			_params._mesh._vertices,
			weights);
	}
	else if (_params._deformationType == DeformationType::MLC)
	{
		calculateMaximumLikelihoodCoordinates(_params._cage._vertices.transpose(),
			_params._cage._faces.transpose(),
			_params._mesh._vertices.transpose(),
			weights);
	}
	else if (_params._deformationType == DeformationType::MEC)
	{
		calculateMaximumEntropyCoordinates(_params._cage._vertices.transpose(),
			_params._cage._faces.transpose(),
			_params._mesh._vertices.transpose(),
			weights,
			1);
	}
	else if (_params._deformationType == DeformationType::BBW)
	{
		// Only a few iterations for sake of demo.
		igl::BBWData bbwData;
		bbwData.active_set_params.max_iter = _params._numBBWSteps;
		bbwData.verbosity = 2;

		if (!igl::bbw(_params._embedding->_vertices, _params._embedding->_faces, _params._b, _params._bc, bbwData, weights))
		{
			LOG_ERROR("Failed to compute bounded biharmonic weights!.");

			return ExecutionResult("Failed to compute bounded biharmonic weights");
		}
	}

	if (DeformationTypeHelpers::RequiresEmbedding(_params._deformationType))
	{
		weights  = (weights.array().colwise() / weights.array().rowwise().sum()).eval();
	}

	LOG_DEBUG("Done computing weights.");

	LOG_DEBUG("Calculating M.");

	// Precompute linear blend skinning matrix.
	Eigen::MatrixXd M;
	if (_params._deformationType == DeformationType::Green ||
		_params._deformationType == DeformationType::QMVC ||
		_params._deformationType == DeformationType::QGC)
	{
		M = weights;
	}
	else if (DeformationTypeHelpers::RequiresEmbedding(_params._deformationType))
	{
		if (_params._interpolateWeights)
		{
			interpolateWeightsInEmbedding(_params._mesh._vertices,
				weights,
				_params._embedding->_vertices,
				_params._embedding->_faces,
				static_cast<int>(_params._cage._vertices.rows()),
				interpolatedWeights,
				M);
		}
		else
		{
			igl::lbs_matrix(_params._embedding->_vertices, weights, M);
		}
	}

	LOG_DEBUG("Done Calculating M.");

	return MeshComputeWeightsOperationResult { std::move(M),
		std::move(weights),
		std::move(interpolatedWeights),
		std::move(psi),
		std::move(psiTri),
		std::move(psiQuad) };
}
