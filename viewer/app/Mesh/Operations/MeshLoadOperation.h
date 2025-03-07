#pragma once

#include <Mesh/Operations/MeshOperation.h>
#include <Mesh/Operations/MeshWeightsParams.h>

#include <filesystem>

struct MeshLoadOperationParams
{
	MeshLoadOperationParams() = default;
	MeshLoadOperationParams(const DeformationType deformationType,
		const LBC::DataSetup::WeightingScheme LBCWeightingScheme,
		std::filesystem::path meshFilepath,
		std::filesystem::path cageFilepath,
		std::optional<std::filesystem::path> deformedCageFilepath,
		std::optional<std::filesystem::path> weightsFilepath,
		std::optional<std::filesystem::path> embeddingFilepath,
		std::optional<std::filesystem::path> parametersFilepath,
		const int32_t numBBWSteps,
		const int32_t numSamples,
		const float scalingFactor,
		const bool interpolateWeights,
		const bool findOffset,
		const bool noOffset
#if WITH_SOMIGLIANA
		, const double somigNu
		, const std::shared_ptr<green::somig_deformer_3>& somiglianaDeformer
#endif
		)
		: _deformationType(deformationType)
		, _LBCWeightingScheme(LBCWeightingScheme)
		, _meshFilepath(std::move(meshFilepath))
		, _cageFilepath(std::move(cageFilepath))
		, _deformedCageFilepath(std::move(deformedCageFilepath))
		, _weightsFilepath(std::move(weightsFilepath))
		, _embeddingFilepath(std::move(embeddingFilepath))
		, _parametersFilepath(std::move(parametersFilepath))
		, _numBBWSteps(numBBWSteps)
		, _numSamples(numSamples)
		, _scalingFactor(scalingFactor)
		, _interpolateWeights(interpolateWeights)
		, _findOffset(findOffset)
		, _noOffset(noOffset)
#if WITH_SOMIGLIANA
		, _somigNu(somigNu)
#endif
	{ }

	[[nodiscard]] bool IsFBX() const
	{
		return _meshFilepath.extension() == ".fbx";
	}

	[[nodiscard]] bool CanInterpolateWeights() const
	{
		return _interpolateWeights && DeformationTypeHelpers::RequiresEmbedding(_deformationType);
	}

	[[nodiscard]] bool HasNoOffset() const
	{
		return _noOffset || _interpolateWeights;
	}

	[[nodiscard]] bool ShouldTriangulateQuads() const
	{
		return _deformationType != DeformationType::QGC && _deformationType != DeformationType::QMVC;
	}

	DeformationType _deformationType = DeformationType::Green;
	LBC::DataSetup::WeightingScheme _LBCWeightingScheme = LBC::DataSetup::WeightingScheme::SQUARE;

	std::filesystem::path _meshFilepath { };
	std::filesystem::path _cageFilepath { };
   
    //std::optional<std::filesystem::path> _cageFilepath { };
    std::optional<std::filesystem::path> _deformedCageFilepath { };
	std::optional<std::filesystem::path> _weightsFilepath { };
	std::optional<std::filesystem::path> _embeddingFilepath { };
	std::optional<std::filesystem::path> _parametersFilepath { };

	int32_t _numBBWSteps = 300;
	int32_t _numSamples = 2;

	float _scalingFactor = 1.0f;

	/// Interpolate the weights.
	bool _interpolateWeights = false;
	bool _findOffset = false;
	bool _noOffset = false;

#if WITH_SOMIGLIANA
	double _somigNu = 0;
#endif
};

class MeshLoadOperation final : public MeshOperationTemplated<MeshLoadOperationParams, std::shared_ptr<ProjectData>>
{
public:
	using MeshOperationTemplated::MeshOperationTemplated;

	[[nodiscard]] std::string GetDescription() const override
	{
		return "Loading mesh data";
	}

	ExecutionResult Execute();
};
