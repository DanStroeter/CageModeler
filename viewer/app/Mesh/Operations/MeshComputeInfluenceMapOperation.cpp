#include <Mesh/Operations/MeshComputeInfluenceMapOperation.h>

#include <cagedeformations/InfluenceMap.h>

MeshComputeInfluenceMapOperation::ExecutionResult MeshComputeInfluenceMapOperation::Execute()
{
	std::vector<int> controlVerticesIdx;
	controlVerticesIdx.reserve(_params._parametrization.translations_per_vertex.size());

	for (const auto& it : _params._parametrization.translations_per_vertex)
	{
		controlVerticesIdx.push_back(it.first);
	}

	bool usesSomigliana = false;

#ifdef WITH_SOMIGLIANA
	usesSomigliana = (_params._deformationType == DeformationType::Somigliana || _params._deformationType == DeformationType::MVC);
#endif

	const Eigen::MatrixXd* weights = nullptr;
	int cageVerticesOffset = 0;
	bool transposeW = false;

	if (!usesSomigliana)
	{
		weights = _params._interpolateWeights ? &_params._weightsData.get()._interpolatedWeights : &_params._weightsData.get()._weights;
		cageVerticesOffset = (_params._deformationType == DeformationType::Green ||
			_params._deformationType == DeformationType::QGC ||
			_params._deformationType == DeformationType::MLC ||
			_params._deformationType == DeformationType::MEC ||
			_params._interpolateWeights) ? 0 : _params._modelVerticesOffset;
		transposeW = _params._deformationType == DeformationType::Green ||
			_params._deformationType == DeformationType::QGC ||
			_params._deformationType == DeformationType::MLC ||
			_params._deformationType == DeformationType::MEC;
	}
	else
	{
#if WITH_SOMIGLIANA
		weights = &_params._somiglianaDeformer->getPhi();
		cageVerticesOffset = 0;
		transposeW = true;
#endif
	}

	Eigen::MatrixXd vertexColors;
	Eigen::VectorXd influences(_params._vertices.rows());
	vertexColors.resize(_params._vertices.rows(), 3);
	for (auto i = 0; i < _params._vertices.rows(); ++i)
	{
		const auto embeddingIndex = i + cageVerticesOffset;
		double res = 0.0;

		if (transposeW)
		{
			for (const auto idx : controlVerticesIdx)
			{
				res += (*weights)(idx, embeddingIndex);
			}
		}
		else
		{
			for (const auto idx : controlVerticesIdx)
			{
				res += (*weights)(embeddingIndex, idx);
			}
		}

		influences(i) = res;
	}

	const auto interpolate = [](double val)
	{
		val = std::min(std::max(0.0, val), 1.0) * 100;
		val = std::log(1 + val) / std::log(1 + 100);
		assert(val >= 0.0 && val <= 1.0);

		return val;
	};

	for (auto i = 0; i < _params._vertices.rows(); ++i)
	{
		vertexColors.row(i) = HSVtoRGB(240.0 * (1.0 - interpolate(influences(i))), 100.0, 100.0);
	}

	return MeshComputeInfluenceMapOperationResult { std::move(vertexColors) };
}
