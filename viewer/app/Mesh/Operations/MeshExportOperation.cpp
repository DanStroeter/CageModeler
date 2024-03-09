#include <Mesh/Operations/MeshExportOperation.h>

#include <igl/writeOBJ.h>

void DeformedMeshExportOperation::Execute()
{
	const auto numSamples = _params._deformationData._vertexData.size();
	const auto deformationTypeVariant = DeformationTypeHelpers::DeformationTypeToString(_params._deformationType, _params._LBCScheme);

	for (auto i = 0; i < numSamples; ++i)
	{
		const auto interpolationFactor = numSamples > 1 ? static_cast<float>(i) / static_cast<float>(numSamples - 1) : 1;
		const auto middle = std::string("_") + std::to_string(interpolationFactor) + std::string("_");

		auto outputFilename = _params._outputFilename.string();
		outputFilename.append(middle);
		outputFilename.append(deformationTypeVariant);
		outputFilename.append(".obj");

#ifdef WITH_SOMIGLIANA
		if (_params._deformationType == DeformationType::MVC || _params._deformationType == DeformationType::Somigliana)
		{
			_params._somiglianaDeformer->save_mesh(outputFilename.c_str());
		}
		else
#endif
		{
			// Scale the output data before we do the export.
			GeometryUtils::ScaleEigenMesh(_params._deformationData._vertexData[i]._vertices, _params._scalingFactor);

			igl::writeOBJ(outputFilename, _params._deformationData._vertexData[i]._vertices, _params._faces);
		}

		LOG_DEBUG("Exported deformed mesh to {}.", outputFilename);
	}
}

void MeshExportOperation::Execute()
{
	const auto deformationTypeVariant = DeformationTypeHelpers::DeformationTypeToString(_params._deformationType, _params._LBCScheme);

	auto outputFilename = _params._outputFilename.string();
	outputFilename.append(deformationTypeVariant);
	outputFilename.append(".obj");

	igl::writeOBJ(outputFilename, _params._vertices, _params._faces);

	LOG_DEBUG("Exported mesh to {}.", outputFilename);
}

