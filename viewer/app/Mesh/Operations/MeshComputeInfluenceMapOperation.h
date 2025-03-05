#pragma once

#include <Mesh/Operations/MeshOperation.h>
#include <Mesh/Operations/MeshWeightsParams.h>

struct MeshComputeInfluenceMapOperationParams
{
	MeshComputeInfluenceMapOperationParams(const DeformationType deformationType,
		const LBC::DataSetup::WeightingScheme LBCScheme,
		const std::shared_ptr<somig_deformer_3>& somiglianaDeformer,
		Eigen::MatrixXd vertices,
		Parametrization parametrization,
		const MeshComputeWeightsOperationResult& weightsData,
		const int32_t modelVerticesOffset,
		const bool interpolateWeights)
		: _deformationType(deformationType)
		, _LBCScheme(LBCScheme)
		, _somiglianaDeformer(somiglianaDeformer)
		, _vertices(std::move(vertices))
		, _parametrization(std::move(parametrization))
		, _weightsData(weightsData)
		, _modelVerticesOffset(modelVerticesOffset)
		, _interpolateWeights(interpolateWeights)
	{ }

	DeformationType _deformationType;
	LBC::DataSetup::WeightingScheme _LBCScheme;

	std::shared_ptr<somig_deformer_3> _somiglianaDeformer = nullptr;

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
