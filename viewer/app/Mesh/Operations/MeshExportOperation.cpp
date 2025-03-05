#include <Mesh/Operations/MeshExportOperation.h>

#include <igl/writeOBJ.h>

void DeformedMeshExportOperation::Execute()
{
	const auto deformationTypeVariant = DeformationTypeHelpers::DeformationTypeToString(_params._deformationType, _params._LBCScheme);

	if (_params._frameIndex.has_value())
	{
		ExportSample(_params._frameIndex.value(), deformationTypeVariant);
	}
	else
	{
		const auto numSamples = _params._deformationData._vertexData.size();

		for (auto i = 0; i < numSamples; ++i)
		{
			ExportSample(i, deformationTypeVariant);
		}
	}
}

void DeformedMeshExportOperation::ExportSample(const std::size_t frameIndex, const std::string_view deformationTypeVariant)
{
	const auto numSamples = _params._deformationData._vertexData.size();
	const auto interpolationFactor = numSamples > 1 ? static_cast<float>(frameIndex) / static_cast<float>(numSamples - 1) : 1;
	const auto middle = std::string("_") + std::to_string(interpolationFactor) + std::string("_");

	auto outputFilename = _params._outputFilename.string();
	outputFilename.append(middle);
	outputFilename.append(deformationTypeVariant);
	outputFilename.append(".obj");

	// Scale the output data before we do the export.
	GeometryUtils::ScaleEigenMesh(_params._deformationData._vertexData[frameIndex]._vertices, _params._scalingFactor);

	igl::writeOBJ(outputFilename, _params._deformationData._vertexData[frameIndex]._vertices, _params._faces);

	LOG_DEBUG("Exported deformed mesh to {}.", outputFilename);
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

