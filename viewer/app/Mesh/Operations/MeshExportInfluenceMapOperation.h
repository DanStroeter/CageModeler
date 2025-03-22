#pragma once

#include <Mesh/Operations/MeshOperation.h>
#include <Mesh/Operations/MeshWeightsParams.h>

#include <filesystem>

struct MeshExportInfluenceMapOperationParams
{
	MeshExportInfluenceMapOperationParams(const DeformationType deformationType,
		const LBC::DataSetup::WeightingScheme LBCScheme,
		const std::shared_ptr<somig_deformer_3>& somiglianaDeformer,
		EigenMesh mesh,
		EigenMesh cage,
		Parametrization parametrization,
		std::filesystem::path outputFilename,
		const MeshComputeWeightsOperationResult& weightsData,
		const int32_t modelVerticesOffset,
		const bool interpolateWeights)
		: _deformationType(deformationType)
		, _LBCScheme(LBCScheme)
		, _somiglianaDeformer(somiglianaDeformer)
		, _mesh(std::move(mesh))
		, _cage(std::move(cage))
		, _parametrization(std::move(parametrization))
		, _outputFilepath(std::move(outputFilename))
		, _weightsData(weightsData)
		, _modelVerticesOffset(modelVerticesOffset)
		, _interpolateWeights(interpolateWeights)
	{ }

	DeformationType _deformationType;
	LBC::DataSetup::WeightingScheme _LBCScheme;

	std::shared_ptr<somig_deformer_3> _somiglianaDeformer = nullptr;

	EigenMesh _mesh;
	EigenMesh _cage;
	Parametrization _parametrization { };
	std::filesystem::path _outputFilepath;
	std::reference_wrapper<const MeshComputeWeightsOperationResult> _weightsData;
	int32_t _modelVerticesOffset;
	uint32_t _interpolateWeights : 1;
};

/**
 * Operation that will export the influence map into a file.
 */
class MeshExportInfluenceMapOperation final : public MeshOperationTemplated<MeshExportInfluenceMapOperationParams, void>
{
public:
	using MeshOperationTemplated::MeshOperationTemplated;

	[[nodiscard]] std::string GetDescription() const override
	{
		return "Exporting influence color map";
	}

	void Execute();
};