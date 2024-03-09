#pragma once

#include <Mesh/Operations/MeshWeightsParams.h>

#ifdef WITH_SOMIGLIANA
	#include <somigliana/somigliana_3d.h>
#endif

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

#ifdef WITH_SOMIGLIANA
		_somiglianaDeformer = other._somiglianaDeformer;
		_somigNu = other._somigNu;
		_somigBulging.store(other._somigBulging.load(std::memory_order_relaxed), std::memory_order_relaxed);
		_somigBlendFactor.store(other._somigBlendFactor.load(std::memory_order_relaxed), std::memory_order_relaxed);
		_somigBulgingType.store(other._somigBulgingType.load(std::memory_order_relaxed), std::memory_order_relaxed);
#endif

		_scalingFactor = other._scalingFactor;
		_useCageAsDeformedCage = other._useCageAsDeformedCage;
		_interpolateWeights = other._interpolateWeights;
		_findOffset = other._findOffset;
		_noOffset = other._noOffset;
		_renderInfluenceMap = other._renderInfluenceMap;
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

#ifdef WITH_SOMIGLIANA
		swap(lhs._somiglianaDeformer, rhs._somiglianaDeformer);
		swap(lhs._somigNu, rhs._somigNu);

		rhs._somigBulging.exchange(lhs._somigBulging);
		rhs._somigBlendFactor.exchange(lhs._somigBlendFactor);
		rhs._somigBulgingType.exchange(lhs._somigBulgingType);
#endif

		swap(lhs._scalingFactor, rhs._scalingFactor);
		swap(lhs._useCageAsDeformedCage, rhs._useCageAsDeformedCage);
		swap(lhs._interpolateWeights, rhs._interpolateWeights);
		swap(lhs._findOffset, rhs._findOffset);
		swap(lhs._noOffset, rhs._noOffset);
		swap(lhs._renderInfluenceMap, rhs._renderInfluenceMap);
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
#ifdef WITH_SOMIGLIANA
			lhs._somiglianaDeformer == rhs._somiglianaDeformer &&
			lhs._somigNu == rhs._somigNu &&
			lhs._somigBulging.load(std::memory_order_relaxed) == rhs._somigBulging.load(std::memory_order_relaxed) &&
			lhs._somigBlendFactor.load(std::memory_order_relaxed) == rhs._somigBlendFactor.load(std::memory_order_relaxed) &&
			lhs._somigBulgingType.load(std::memory_order_relaxed) == rhs._somigBulgingType.load(std::memory_order_relaxed) &&
#endif
			lhs._scalingFactor == rhs._scalingFactor &&
			lhs._useCageAsDeformedCage == rhs._useCageAsDeformedCage &&
			lhs._interpolateWeights == rhs._interpolateWeights &&
			lhs._findOffset == rhs._findOffset &&
			lhs._noOffset == rhs._noOffset;
	}

	[[nodiscard]] bool friend operator!=(const ProjectModelData& lhs, const ProjectModelData& rhs)
	{
		return !(lhs == rhs);
	}

	[[nodiscard]] bool CanEditInfluenceMapSetting() const
	{
#if WITH_SOMIGLIANA
		return (_deformationType != DeformationType::Somigliana && _deformationType != DeformationType::MVC);
#else
		return true;
#endif
	}

	[[nodiscard]] bool CanRenderInfluenceMap() const
	{
#if WITH_SOMIGLIANA
		return (_deformationType != DeformationType::Somigliana && _deformationType != DeformationType::MVC) && _renderInfluenceMap;
#else
		return _renderInfluenceMap;
#endif
	}

#if WITH_SOMIGLIANA
	[[nodiscard]] double GetSomiglianaBulging() const
	{
		return _somigBulging.load(std::memory_order_relaxed);
	}

	[[nodiscard]] double GetSomiglianaBlendFactor() const
	{
		return _somigBlendFactor.load(std::memory_order_relaxed);
	}

	[[nodiscard]] BulgingType GetSomiglianaBulgingType() const
	{
		return _somigBulgingType.load(std::memory_order_relaxed);
	}
#endif

	DeformationType _deformationType = DeformationType::Green;
	LBC::DataSetup::WeightingScheme _LBCWeightingScheme = LBC::DataSetup::WeightingScheme::SQUARE;

	int32_t _numBBWSteps = 300;
	int32_t _numSamples = 1;

	std::optional<std::filesystem::path> _meshFilepath;
	std::optional<std::filesystem::path> _weightsFilepath;
	std::optional<std::filesystem::path> _cageFilepath;
	std::optional<std::filesystem::path> _embeddingFilepath;
	std::optional<std::filesystem::path> _deformedCageFilepath;
	std::optional<std::filesystem::path> _parametersFilepath;

#ifdef WITH_SOMIGLIANA
	std::shared_ptr<green::somig_deformer_3> _somiglianaDeformer = nullptr;

	double _somigNu = 0;
	std::atomic<double> _somigBulging = 0;
	std::atomic<double> _somigBlendFactor = 0;
	std::atomic<BulgingType> _somigBulgingType = SWEPT_VOLUME;
#endif

	float _scalingFactor = 1.0f;

	/// Use the object cage as the base for the deformed cage.
	bool _useCageAsDeformedCage = false;

	/// Interpolate the weights.
	bool _interpolateWeights = false;
	bool _findOffset = false;
	bool _noOffset = false;

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
