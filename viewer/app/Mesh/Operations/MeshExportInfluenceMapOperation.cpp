#include <Mesh/Operations/MeshExportInfluenceMapOperation.h>

#include <cagedeformations/InfluenceMap.h>

void MeshExportInfluenceMapOperation::Execute()
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

	if (!usesSomigliana)
	{
		const auto& weights = _params._interpolateWeights ? _params._weightsData.get()._interpolatedWeights : _params._weightsData.get()._weights;
		const auto cageVerticesOffset = (_params._deformationType == DeformationType::Green ||
			_params._deformationType == DeformationType::QGC ||
			_params._deformationType == DeformationType::MLC ||
			_params._deformationType == DeformationType::MEC ||
			_params._interpolateWeights) ? 0 : _params._modelVerticesOffset;
		const auto transposeW = _params._deformationType == DeformationType::Green ||
			_params._deformationType == DeformationType::QGC ||
			_params._deformationType == DeformationType::MLC ||
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
#if WITH_SOMIGLIANA
		write_influence_color_map_OBJ(_params._outputFilepath.string(),
			_params._mesh._vertices,
			_params._mesh._faces,
			_params._somiglianaDeformer->getPhi(),
			controlVerticesIdx,
			0,
			true);
#endif
	}
}
