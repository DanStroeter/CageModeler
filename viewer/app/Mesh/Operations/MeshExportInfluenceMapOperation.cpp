#include <Mesh/Operations/MeshExportInfluenceMapOperation.h>

#include <cagedeformations/InfluenceMap.h>

void MeshExportInfluenceMapOperation::Execute()
{
	std::vector<int> controlVerticesIdx;
	if (_params._selectedVertices.has_value())
	{
		controlVerticesIdx.reserve(_params._selectedVertices->size());
		for (const auto vertexIdx : _params._selectedVertices.value())
		{
			controlVerticesIdx.push_back(vertexIdx);
		}
	}
	else
	{
		controlVerticesIdx.reserve(_params._parametrization.translations_per_vertex.size());
		for (const auto& it : _params._parametrization.translations_per_vertex)
		{
			controlVerticesIdx.push_back(it.first);
		}
	}

	bool usesSomigliana = usesSomigliana = (_params._deformationType == DeformationType::Somigliana);

	if (!usesSomigliana)
	{
		const auto& sourceWeights =
			_params._interpolateWeights
			? _params._weightsData.get()._interpolatedWeights
			: _params._weightsData.get()._weights;

		auto weights = sourceWeights;

		if (_params._deformationType == DeformationType::PMVC)
		{
			weights.transposeInPlace();
		}
		const auto cageVerticesOffset = (_params._deformationType == DeformationType::Green ||
			_params._deformationType == DeformationType::QGC ||
			_params._deformationType == DeformationType::MLC ||
			_params._deformationType == DeformationType::MEC ||
			_params._deformationType == DeformationType::MVC ||
			_params._deformationType == DeformationType::PMVC ||
			_params._deformationType == DeformationType::QMVC ||
			_params._interpolateWeights) ? 0 : _params._modelVerticesOffset;
		const auto transposeW = _params._deformationType == DeformationType::Green ||
			_params._deformationType == DeformationType::QGC ||
			_params._deformationType == DeformationType::MLC ||
			_params._deformationType == DeformationType::MVC ||
			_params._deformationType == DeformationType::PMVC ||
			_params._deformationType == DeformationType::MEC;

		write_influence_color_map_OBJ(_params._outputFilepath.string(),
			_params._mesh._vertices,
			_params._mesh._faces,
			weights,
			controlVerticesIdx,
			cageVerticesOffset,
			transposeW);
	}
	else
	{
		write_influence_color_map_OBJ(_params._outputFilepath.string(),
			_params._mesh._vertices,
			_params._mesh._faces,
			_params._somiglianaDeformer->getPhi(),
			controlVerticesIdx,
			0,
			true);
	}
}
