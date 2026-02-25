#include <Mesh/Operations/MeshComputeDeformationOperation.h>

#include <cagedeformations/WeightInterpolation.h>
#include <cagedeformations/GreenCoordinates.h>

#include <igl/lbs_matrix.h>
#include <igl/EPS.h>
#include <fstream>

void WriteWeightsToFile(const std::string& filename, Eigen::MatrixXd _lambdaResults)
{
	std::ofstream file(filename);
	if (!file.is_open())
		throw std::runtime_error("Failed to open weight write file!");

	const Eigen::Index rows = _lambdaResults.rows();
	const Eigen::Index cols = _lambdaResults.cols();

	file << "===== LAMBDA BUFFER (rows=" << rows
		<< ", cols=" << cols << ") =====\n";

	for (Eigen::Index r = 0; r < rows; ++r)
	{
		double sum = 0.0;

		for (Eigen::Index c = 0; c < cols; ++c)
		{
			const double value = _lambdaResults(r, c);
			file << "lambda[" << r << "][" << c << "] = " << value << "\n";
			sum += value;
		}
	}

	file.close();
	LOG_INFO("Weights written to " + filename);
}


MeshComputeDeformationOperation::ExecutionResult MeshComputeDeformationOperation::Execute()
{
	const auto params = fromDeformedCage(_params._cage._vertices, _params._deformedCage._vertices);

	const auto dim = _params._cage._vertices.cols();
	const auto translationFactors = createRegularSampling(_params._numSamples, params.minFactor, params.maxFactor);
	const auto numControlVertices = _params._cage._vertices.rows();
	std::vector<uint8_t> controlVerticesMarked(numControlVertices, 0);

	for (const auto& [controlVertexIndex, _] : params.translations_per_vertex)
	{
		controlVerticesMarked[controlVertexIndex] = 1;
	}

	const auto numTransformations = numControlVertices;
	Eigen::VectorXi tetTags(_params._mesh._faces.rows());
	for (auto j = 0; j < _params._mesh._faces.rows(); ++j)
	{
		tetTags(j) = 1;
	}

	Eigen::MatrixXd finalTransformation(numTransformations * (dim + 1), dim);
	for (auto j = 0; j < numTransformations; ++j)
	{
		const auto a = Eigen::Affine3d::Identity();
		finalTransformation.block(j * (dim + 1), 0, dim + 1, dim) =
			a.matrix().transpose().block(0, 0, dim + 1, dim);
	}

	std::vector<InterpolatedVertexData> vertexData(translationFactors.size());

	for (auto i = 0; i < translationFactors.size(); ++i)
	{
		LOG_DEBUG("DeformationType = {}, sample {} / {}",
			static_cast<int>(_params._deformationType),
			i,
			translationFactors.size());

		auto cageTransforms = finalTransformation;
		_params._deformedCage._vertices = _params._cage._vertices;
		const auto factor = translationFactors[i];

		for (const auto& [cageVertexIndex, position] : params.translations_per_vertex)
		{
			const auto translation = Eigen::Affine3d(Eigen::Translation3d(factor * position));
			finalTransformation.block(cageVertexIndex * (dim + 1), 0, dim + 1, dim) = translation.matrix().transpose().block(0, 0, dim + 1, dim);
			const Eigen::Vector3d cageVertex = _params._cage._vertices.row(cageVertexIndex);
			_params._deformedCage._vertices.row(cageVertexIndex) = cageVertex + factor * position;
		}

		Eigen::MatrixXd U;

		// Allocate the space for the vertices.
		vertexData[i]._vertices = Eigen::MatrixXd(_params._mesh._vertices.rows(), 3);
		vertexData[i]._translationFactor = factor;

		if (i == 0)
		{
			LOG_DEBUG("CAGE verts: {} x {}",
				_params._cage._vertices.rows(),
				_params._cage._vertices.cols());

			LOG_DEBUG("DEF CAGE verts: {} x {}",
				_params._deformedCage._vertices.rows(),
				_params._deformedCage._vertices.cols());
			LOG_DEBUG("WEIGHTS matrix: {} x {}",
				_params._weightsData._weights.rows(),
				_params._weightsData._weights.cols());

			LOG_DEBUG("WEIGHTS^T matrix: {} x {}",
				_params._weightsData._weights.cols(),
				_params._weightsData._weights.rows());
		}

		if (_params._deformationType == DeformationType::Green)
		{
			
			std::cout << std::endl;
			
			Eigen::MatrixXd deformedNormals;
			calcNormals(_params._deformedCage._vertices, _params._cage._faces, deformedNormals);
			calcScalingFactors(_params._cage._vertices, _params._deformedCage._vertices, _params._cage._faces, deformedNormals);
			vertexData[i]._vertices = _params._weightsData._weights.transpose() * _params._deformedCage._vertices + _params._weightsData._psi.transpose() * deformedNormals;
		}
		else if (_params._deformationType == DeformationType::QGC)
		{
			calcNewPositionsTriQuad(_params._cage._vertices,
				_params._deformedCage._vertices,
				_params._cage._faces,
				_params._weightsData._weights,
				_params._weightsData._psiTri,
				_params._weightsData._psiQuad,
				vertexData[i]._vertices);
		}
		else if (_params._deformationType == DeformationType::MVC ||_params._deformationType == DeformationType::QMVC || _params._deformationType == DeformationType::MLC ||
			_params._deformationType == DeformationType::MEC)
		{
			WriteWeightsToFile("mvc", _params._weightsData._weights.transpose());
			vertexData[i]._vertices = _params._weightsData._weights.transpose() * _params._deformedCage._vertices;
		}
		else if (_params._deformationType == DeformationType::PMVCRayracing || _params._deformationType == DeformationType::PMVCLipman) {
			/*vertexData[i]._vertices =
				_params._weightsData._weights *
				_params._deformedCage._vertices;*/
			/*const auto& C0 = _params._cage._vertices;
			const auto& V0 = _params._mesh._vertices;

			// Deformed cage
			const auto& C1 = _params._deformedCage._vertices;

			// PMVC weights
			const auto& W = _params._weightsData._weights;

			// 1) Project rest mesh to rest cage
			Eigen::MatrixXd proj0 = W * C0;

			// 2) Compute offset
			Eigen::MatrixXd offset = V0 - proj0;

			// 3) Project onto deformed cage
			Eigen::MatrixXd proj1 = W * C1;

			// 4) Final vertices
			vertexData[i]._vertices = proj1 + offset;*/
			WriteWeightsToFile("pmvc", _params._weightsData._weights);
			vertexData[i]._vertices = _params._weightsData._weights * _params._deformedCage._vertices;
		}
		else if (_params._deformationType == DeformationType::Somigliana)
		{
			std::vector<point3d> verts_deformed_somig(_params._mesh._vertices.rows()), cage_deformed_somig(_params._cage._vertices.rows());

			for (Eigen::Index j = 0; j < _params._deformedCage._vertices.rows(); ++j)
			{
				cage_deformed_somig[j] = { _params._deformedCage._vertices(j, 0), _params._deformedCage._vertices(j, 1), _params._deformedCage._vertices(j, 2) };
			}

			_params._somiglianaDeformer->deform(cage_deformed_somig, verts_deformed_somig);

			for (Eigen::Index j = 0; j < _params._mesh._vertices.rows(); ++j) 
			{
				auto const vert = verts_deformed_somig[j];
				vertexData[i]._vertices.row(j) = Eigen::Vector3d(vert.x(), vert.y(), vert.z());
			}
		}
		else
		{
			U = _params._weightsData._skinningMatrix * finalTransformation;
		}

		if (DeformationTypeHelpers::RequiresEmbedding(_params._deformationType))
		{
			for (Eigen::Index j = 0; j < _params._mesh._vertices.rows(); ++j)
			{
				vertexData[i]._vertices.row(j) = U.row(j + _params._modelVerticesOffset);
			}
		}
	}

	return MeshComputeDeformationOperationResult { std::move(vertexData) };
}
