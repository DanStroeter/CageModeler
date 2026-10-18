#pragma once

#include <Mesh/Operations/MeshWeightsParams.h>
#include <cagedeformations/somig.h>
#include <Mesh/Operations/MeshCageGeneration.h>

#include <algorithm>

struct ProjectModelData
{
	ProjectModelData() = default;
	~ProjectModelData() noexcept = default;

	ProjectModelData(const ProjectModelData& other)
	{
		_deformationType = other._deformationType;
		_LBCWeightingScheme = other._LBCWeightingScheme;
		_numBBWSteps = other._numBBWSteps;
		_numSamples = other._numSamples;
		_meshFilepath = other._meshFilepath;
		_weightsFilepath = other._weightsFilepath;
		_cageFilepath = other._cageFilepath;
		_embeddingFilepath = other._embeddingFilepath;
		_deformedCageFilepath = other._deformedCageFilepath;
		_parametersFilepath = other._parametersFilepath;
		_somiglianaDeformer = other._somiglianaDeformer;
		_somigNu = other._somigNu;
		_scalingFactor = other._scalingFactor;
		_interpolateWeights = other._interpolateWeights;
		_findOffset = other._findOffset;
		_noOffset = other._noOffset;
		_pmvcUseOffset = other._pmvcUseOffset;
		_renderInfluenceMap = other._renderInfluenceMap;
		_pmvcHitCount = other._pmvcHitCount;
		_pmvcSkipEvenHits = other._pmvcSkipEvenHits;
		_pmvcUseInteriorDistance = other._pmvcUseInteriorDistance;
	}

	ProjectModelData(ProjectModelData&& other) noexcept
		: ProjectModelData()
	{
		swap(*this, other);
	}

	ProjectModelData& operator=(const ProjectModelData& other)
	{
		ProjectModelData temp(other);
		swap(*this, temp);

		return *this;
	}

	ProjectModelData& operator=(ProjectModelData&& other) noexcept
	{
		swap(*this, other);

		return *this;
	}

	friend void swap(ProjectModelData& lhs, ProjectModelData& rhs) noexcept
	{
		using std::swap;

		swap(lhs._deformationType, rhs._deformationType);
		swap(lhs._LBCWeightingScheme, rhs._LBCWeightingScheme);
		swap(lhs._numBBWSteps, rhs._numBBWSteps);
		swap(lhs._numSamples, rhs._numSamples);
		swap(lhs._meshFilepath, rhs._meshFilepath);
		swap(lhs._weightsFilepath, rhs._weightsFilepath);
		swap(lhs._cageFilepath, rhs._cageFilepath);
		swap(lhs._embeddingFilepath, rhs._embeddingFilepath);
		swap(lhs._deformedCageFilepath, rhs._deformedCageFilepath);
		swap(lhs._parametersFilepath, rhs._parametersFilepath);
		swap(lhs._somiglianaDeformer, rhs._somiglianaDeformer);
		swap(lhs._somigNu, rhs._somigNu);
		swap(lhs._scalingFactor, rhs._scalingFactor);
		swap(lhs._interpolateWeights, rhs._interpolateWeights);
		swap(lhs._findOffset, rhs._findOffset);
		swap(lhs._noOffset, rhs._noOffset);
		swap(lhs._pmvcUseOffset, rhs._pmvcUseOffset);
		swap(lhs._renderInfluenceMap, rhs._renderInfluenceMap);
		swap(lhs._pmvcHitCount, rhs._pmvcHitCount);
		swap(lhs._pmvcSkipEvenHits, rhs._pmvcSkipEvenHits);
		swap(lhs._pmvcUseInteriorDistance, rhs._pmvcUseInteriorDistance);
	}

	[[nodiscard]] bool IsFBX() const
	{
		return _meshFilepath->extension() == ".fbx";
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

	[[nodiscard]] bool friend operator==(const ProjectModelData& lhs, const ProjectModelData& rhs)
	{
		return lhs._deformationType == rhs._deformationType &&
			lhs._LBCWeightingScheme == rhs._LBCWeightingScheme &&
			lhs._numBBWSteps == rhs._numBBWSteps &&
			lhs._numSamples == rhs._numSamples &&
			lhs._meshFilepath == rhs._meshFilepath &&
			lhs._weightsFilepath == rhs._weightsFilepath &&
			lhs._cageFilepath == rhs._cageFilepath &&
			lhs._embeddingFilepath == rhs._embeddingFilepath &&
			lhs._deformedCageFilepath == rhs._deformedCageFilepath &&
			lhs._parametersFilepath == rhs._parametersFilepath &&
			lhs._somiglianaDeformer == rhs._somiglianaDeformer &&
			lhs._somigNu == rhs._somigNu &&
			lhs._scalingFactor == rhs._scalingFactor &&
			lhs._interpolateWeights == rhs._interpolateWeights &&
			lhs._findOffset == rhs._findOffset &&
			lhs._noOffset == rhs._noOffset &&
			lhs._pmvcUseOffset == rhs._pmvcUseOffset &&
			lhs._pmvcHitCount == rhs._pmvcHitCount &&
			lhs._pmvcSkipEvenHits == rhs._pmvcSkipEvenHits &&
			lhs._pmvcUseInteriorDistance == rhs._pmvcUseInteriorDistance;
	}

	[[nodiscard]] bool friend operator!=(const ProjectModelData& lhs, const ProjectModelData& rhs)
	{
		return !(lhs == rhs);
	}

	[[nodiscard]] bool CanEditInfluenceMapSetting() const
	{
		return _deformationType != DeformationType::Somigliana;
	}

	[[nodiscard]] bool CanRenderInfluenceMap() const
	{
		return _deformationType != DeformationType::Somigliana && _renderInfluenceMap;
	}

	/**
	 * Keeps the PMVC settings consistent with the selected coordinate type. The offset
	 * variant is not a setting of its own anymore, it is the PMVCO coordinate type, and
	 * because it weights by solid angle alone it has no per-ray split: it runs with a single
	 * hit and none of the settings that shape that split apply to it.
	 */
	void ApplyPMVCPreset()
	{
		if (_deformationType == DeformationType::PMVC)
		{
			_pmvcUseOffset = false;
			_pmvcHitCount = std::clamp<uint64_t>(_pmvcHitCount, 1, PMVCSettings::kMaxHitCount);
		}
		else if (_deformationType == DeformationType::PMVCO)
		{
			_pmvcUseOffset = true;
			_pmvcHitCount = 1;
			_pmvcSkipEvenHits = false;
			_pmvcUseInteriorDistance = false;
		}
	}

	/**
	 * @return Check if the files exist, otherwise we will end up with errors.
	 */
	[[nodiscard]] bool CheckMissingFiles() const
	{
		const auto hasNoMeshFile = (_meshFilepath.has_value() && !std::filesystem::exists(_meshFilepath.value()));
		const auto hasNoCageFile = (_cageFilepath.has_value() && !std::filesystem::exists(_cageFilepath.value()));
		const auto hasNoDeformedCageFile = (_deformedCageFilepath.has_value() && !std::filesystem::exists(_deformedCageFilepath.value()));
		const auto hasNoWeightsFile = (_weightsFilepath.has_value() && !std::filesystem::exists(_weightsFilepath.value()));
		const auto hasNoParamsFile = (_parametersFilepath.has_value() && !std::filesystem::exists(_parametersFilepath.value()));
		const auto hasNoEmbedding = (DeformationTypeHelpers::RequiresEmbedding(_deformationType) && _embeddingFilepath.has_value() && !std::filesystem::exists(_embeddingFilepath.value()));

		return hasNoMeshFile || hasNoCageFile || hasNoWeightsFile || hasNoDeformedCageFile || hasNoParamsFile || hasNoEmbedding;
	}

	DeformationType _deformationType = DeformationType::Green;
	LBC::DataSetup::WeightingScheme _LBCWeightingScheme = LBC::DataSetup::WeightingScheme::SQUARE;
	CageGenerationMethod _cageGenerationType = CageGenerationMethod::CoarseBoundingCagesByXian;

	int32_t _numBBWSteps = 300;
	int32_t _numSamples = 2;

	std::optional<std::filesystem::path> _meshFilepath;
	std::optional<std::filesystem::path> _weightsFilepath;
	std::optional<std::filesystem::path> _cageFilepath;
	std::optional<std::filesystem::path> _embeddingFilepath;
	std::optional<std::filesystem::path> _deformedCageFilepath;
	std::optional<std::filesystem::path> _parametersFilepath;

	/// Number of depth peeling layers rendered per mesh vertex. All of them are weighted
	/// against each other along their ray and normalized, so every ray contributes the
	/// same leverage regardless of how often it crosses the cage.
	uint64_t _pmvcHitCount = 1;

	/// Drop the entry (every second) hits from the per-ray weight split.
	bool _pmvcSkipEvenHits = false;

	/// Bias the per-ray weight split towards hits that are reachable without a detour
	/// through the cage interior. Ignored by the offset (PMVCO) variant.
	bool _pmvcUseInteriorDistance = false;

	std::shared_ptr<somig_deformer_3> _somiglianaDeformer = nullptr;

	double _somigNu = 0;

	float _scalingFactor = 1.0f;

	/// Interpolate the weights.
	bool _interpolateWeights = false;
	bool _findOffset = false;
	bool _noOffset = false;

	/// Derived from the coordinate type (PMVCO), not an independent setting.
	bool _pmvcUseOffset = false;

	/// Render the influence of the mesh as vertex colors.
	bool _renderInfluenceMap = false;
};

template <typename T>
class UIDoubleBuffer
{
public:
	[[nodiscard]] const T& Read() const
	{
		return _data[_activeIndex];
	}

	[[nodiscard]] T& operator->()
	{
		return _data[_activeIndex & 2];
	}

	void Swap()
	{
		std::swap(_data[0], _data[1]);

		_activeIndex = _activeIndex & 2;
	}

	[[nodiscard]] bool IsEqual() const
	{
		return _data[0] == _data[1];
	}

private:
	T _data[2];
	std::size_t _activeIndex = 0;
};

using ProjectModel = UIDoubleBuffer<std::shared_ptr<ProjectModelData>>;
