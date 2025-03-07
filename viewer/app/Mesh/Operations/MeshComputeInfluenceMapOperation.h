#pragma once

#include <Mesh/Operations/MeshOperation.h>
#include <Mesh/Operations/MeshWeightsParams.h>

struct MeshComputeInfluenceMapOperationParams
{
	MeshComputeInfluenceMapOperationParams(const DeformationType deformationType,
		const LBC::DataSetup::WeightingScheme LBCScheme,
#if WITH_SOMIGLIANA
		const std::shared_ptr<green::somig_deformer_3>& somiglianaDeformer,
#endif
		Eigen::MatrixXd vertices,
		Parametrization parametrization,
		const MeshComputeWeightsOperationResult& weightsData,
		const int32_t modelVerticesOffset,
		const bool interpolateWeights)
		: _deformationType(deformationType)
		, _LBCScheme(LBCScheme)
#if WITH_SOMIGLIANA
		, _somiglianaDeformer(somiglianaDeformer)
#endif
		, _vertices(std::move(vertices))
		, _parametrization(std::move(parametrization))
		, _weightsData(weightsData)
		, _modelVerticesOffset(modelVerticesOffset)
		, _interpolateWeights(interpolateWeights)
	{ }

	DeformationType _deformationType;
	LBC::DataSetup::WeightingScheme _LBCScheme;

#if WITH_SOMIGLIANA
	std::shared_ptr<green::somig_deformer_3> _somiglianaDeformer = nullptr;
#endif

	Eigen::MatrixXd _vertices;
	Parametrization _parametrization { };
	std::reference_wrapper<const MeshComputeWeightsOperationResult> _weightsData;
	int32_t _modelVerticesOffset;
	uint32_t _interpolateWeights : 1;
};

struct MeshComputeInfluenceMapOperationResult
{
	Eigen::MatrixXd _vertexColors;
};

class MeshComputeInfluenceMapOperation final : public MeshOperationTemplated<MeshComputeInfluenceMapOperationParams, MeshComputeInfluenceMapOperationResult>
{
public:
	using MeshOperationTemplated::MeshOperationTemplated;

	[[nodiscard]] std::string GetDescription() const override
	{
		return "Exporting influence color map";
	}

	ExecutionResult Execute();
};
