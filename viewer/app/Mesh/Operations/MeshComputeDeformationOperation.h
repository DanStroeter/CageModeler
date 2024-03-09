#pragma once

#include <Mesh/Operations/MeshOperation.h>
#include <Mesh/Operations/MeshWeightsParams.h>

#include <Eigen/Core>

struct MeshComputeDeformationOperationParams
{
	MeshComputeDeformationOperationParams(const DeformationType deformationType,
		const LBC::DataSetup::WeightingScheme LBCScheme,
#if WITH_SOMIGLIANA
		const std::shared_ptr<green::somig_deformer_3>& somiglianaDeformer,
		const double somigBulging,
		const double somigBlendFactor,
		const BulgingType somigBulgingType,
#endif
		EigenMesh mesh,
		EigenMesh cageMesh,
		EigenMesh deformedCage,
		MeshComputeWeightsOperationResult weightsData,
		const int32_t modelVerticesOffset,
		const int32_t numSamples,
		const bool interpolateWeights)
		: _deformationType(deformationType)
		, _LBCScheme(LBCScheme)
#if WITH_SOMIGLIANA
		, _somiglianaDeformer(somiglianaDeformer)
		, _somigBulging(somigBulging)
		, _somigBlendFactor(somigBlendFactor)
		, _somigBulgingType(somigBulgingType)
#endif
		, _mesh(std::move(mesh))
		, _cage(std::move(cageMesh))
		, _deformedCage(std::move(deformedCage))
		, _weightsData(std::move(weightsData))
		, _modelVerticesOffset(modelVerticesOffset)
		, _numSamples(numSamples)
		, _interpolateWeights(interpolateWeights)
	{ }

	DeformationType _deformationType = DeformationType::Green;
	LBC::DataSetup::WeightingScheme _LBCScheme = LBC::DataSetup::WeightingScheme::SQUARE;

#if WITH_SOMIGLIANA
	std::shared_ptr<green::somig_deformer_3> _somiglianaDeformer = nullptr;

	double _somigBulging = 0;
	double _somigBlendFactor = 0;
	BulgingType _somigBulgingType = SWEPT_VOLUME;
#endif

	EigenMesh _mesh;
	EigenMesh _cage;
	EigenMesh _deformedCage;
	MeshComputeWeightsOperationResult _weightsData;
	int32_t _modelVerticesOffset = 0;
	int32_t _numSamples;

	uint32_t _interpolateWeights : 1;
};

class MeshComputeDeformationOperation final : public MeshOperationTemplated<MeshComputeDeformationOperationParams, MeshComputeDeformationOperationResult>
{
public:
	using MeshOperationTemplated::MeshOperationTemplated;

	[[nodiscard]] std::string GetDescription() const override
	{
		return "Computing deformed mesh";
	}

	ExecutionResult Execute();
};
