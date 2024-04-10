#include <Mesh/Operations/MeshComputeDeformationOperation.h>

#include <cagedeformations/WeightInterpolation.h>
#include <cagedeformations/GreenCoordinates.h>

#include <igl/lbs_matrix.h>
#include <igl/EPS.h>

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

		if (_params._deformationType == DeformationType::Green)
		{
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
		else if (_params._deformationType == DeformationType::QMVC || _params._deformationType == DeformationType::MLC || _params._deformationType == DeformationType::MEC)
		{
			vertexData[i]._vertices = _params._weightsData._weights.transpose() * _params._deformedCage._vertices;
		}
#ifdef WITH_SOMIGLIANA
		else if (_params._deformationType == DeformationType::Somigliana)
		{
			_params._somiglianaDeformer->deform(_params._deformedCage._vertices, SOMIGLIANA, _params._somigBulgingType, _params._somigBulging, _params._somigBlendFactor);

			vertexData[i]._vertices = _params._somiglianaDeformer->V_.transpose();
		}
		else if (_params._deformationType == DeformationType::MVC)
		{
			_params._somiglianaDeformer->deform(_params._deformedCage._vertices, MEANVALUE, _params._somigBulgingType, _params._somigBulging, _params._somigBlendFactor);

			vertexData[i]._vertices = _params._somiglianaDeformer->V_.transpose();
		}
#endif
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
