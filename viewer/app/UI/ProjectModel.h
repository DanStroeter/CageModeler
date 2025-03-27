#pragma once

#include <Mesh/Operations/MeshWeightsParams.h>
#include <cagedeformations/somig.h>

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
		_renderInfluenceMap = other._renderInfluenceMap;
		_smoothIterations = other._smoothIterations;
		_targetNumFaces = other._targetNumFaces;
		_closingResult = other._closingResult;
		_voxelResolution = other._voxelResolution;
		
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
		swap(lhs._renderInfluenceMap, rhs._renderInfluenceMap);
		swap(lhs._smoothIterations, rhs._smoothIterations);
		swap(lhs._targetNumFaces, rhs._targetNumFaces);
		swap(lhs._closingResult, rhs._closingResult);
		swap(lhs._voxelResolution, rhs._voxelResolution);
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
			lhs._smoothIterations == rhs._smoothIterations &&
			lhs._targetNumFaces == rhs._targetNumFaces &&
			lhs._closingResult == rhs._closingResult &&
			lhs._voxelResolution == rhs._voxelResolution;
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

	/**
	 * @return Check if the mesh files exist when generate the cage of the mesh, otherwise we will end up with errors.
	 */
	[[nodiscard]] bool CheckMissingFilesForCageGeneration() const
	{
		const auto hasNoMeshFile = (_meshFilepath.has_value() && !std::filesystem::exists(_meshFilepath.value()));
		//const auto hasNoCageFile = (_cageFilepath.has_value() && !std::filesystem::exists(_cageFilepath.value()));
		//const auto hasNoDeformedCageFile = (_deformedCageFilepath.has_value() && !std::filesystem::exists(_deformedCageFilepath.value()));
		//const auto hasNoWeightsFile = (_weightsFilepath.has_value() && !std::filesystem::exists(_weightsFilepath.value()));
		//const auto hasNoParamsFile = (_parametersFilepath.has_value() && !std::filesystem::exists(_parametersFilepath.value()));
		//const auto hasNoEmbedding = (DeformationTypeHelpers::RequiresEmbedding(_deformationType) && _embeddingFilepath.has_value() && !std::filesystem::exists(_embeddingFilepath.value()));

		return hasNoMeshFile;
	}

	DeformationType _deformationType = DeformationType::Green;
	LBC::DataSetup::WeightingScheme _LBCWeightingScheme = LBC::DataSetup::WeightingScheme::SQUARE;

	int32_t _numBBWSteps = 300;
	int32_t _numSamples = 2;

	int32_t _smoothIterations = 3;
	int32_t _targetNumFaces = 400;
	int32_t _voxelResolution = 32;

	std::vector<bool> _closingResult;

	std::optional<std::filesystem::path> _meshFilepath;
	std::optional<std::filesystem::path> _weightsFilepath;
	std::optional<std::filesystem::path> _cageFilepath;
	std::optional<std::filesystem::path> _embeddingFilepath;
	std::optional<std::filesystem::path> _deformedCageFilepath;
	std::optional<std::filesystem::path> _parametersFilepath;

	std::shared_ptr<somig_deformer_3> _somiglianaDeformer = nullptr;

	double _somigNu = 0;

	float _scalingFactor = 1.0f;

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
