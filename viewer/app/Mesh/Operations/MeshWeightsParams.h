#pragma once

#include <Mesh/GeometryUtils.h>

#include <cagedeformations/Parametrization.h>

#include <Eigen/Core>
#include <LBC/DataSetup.h>

#ifdef WITH_SOMIGLIANA
	#include <somigliana/somigliana_3d.h>
#endif

enum class DeformationType : uint8_t
{
#ifdef WITH_SOMIGLIANA
	MVC,
#endif
	QMVC,
	Harmonic,
	BBW,
	LBC,
	MEC,
	MLC,
	Green,
	QGC,
#ifdef WITH_SOMIGLIANA
	Somigliana
#endif
};

struct DeformationTypeHelpers
{
	[[nodiscard]] static std::string DeformationTypeToString(const DeformationType deformationType, const LBC::DataSetup::WeightingScheme LBCScheme)
	{
#ifdef WITH_SOMIGLIANA
		if (deformationType == DeformationType::MVC)
		{
			return "MVC";
		}
		else
#endif
		if (deformationType == DeformationType::QMVC)
		{
			return "QMVC";
		}
		else if (deformationType == DeformationType::Harmonic)
		{
			return "Harmonic";
		}
		else if (deformationType == DeformationType::BBW)
		{
			return "BBW";
		}
		else if (deformationType == DeformationType::LBC)
		{
			return std::string("lbc_") + std::to_string(LBCScheme);
		}
		else if (deformationType == DeformationType::MEC)
		{
			return "MEC";
		}
		else if (deformationType == DeformationType::MLC)
		{
			return "MLC";
		}
		else if (deformationType == DeformationType::Green)
		{
			return "Green";
		}
		else if (deformationType == DeformationType::QGC)
		{
			return "QGC";
		}
#ifdef WITH_SOMIGLIANA
		else if (deformationType == DeformationType::Somigliana)
		{
			return "Somigliana";
		}
#endif

		return { };
	}

	[[nodiscard]] static bool RequiresEmbedding(const DeformationType deformationType)
	{
		return deformationType == DeformationType::LBC || deformationType == DeformationType::Harmonic || deformationType == DeformationType::BBW;
	}
};

template <typename Scalar, int Rows, int Cols>
struct std::hash<Eigen::Matrix<Scalar, Rows, Cols>>
{
	// https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
	size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const
	{
		size_t seed = 0;
		for (auto i = 0; i < matrix.size(); ++i)
		{
			Scalar elem = *(matrix.data() + i);
			seed ^= std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		}

		return seed;
	}
}; // namespace std

struct MeshComputeWeightsOperationResult
{
	void Update(Eigen::MatrixXd skinningMatrix,
		Eigen::MatrixXd weights,
		Eigen::MatrixXd interpolatedWeights,
		Eigen::MatrixXd psi,
		std::vector<double> psiTri,
		std::vector<Eigen::Vector4d> psiQuad)
	{
		_skinningMatrix = std::move(skinningMatrix);
		_weights = std::move(weights);
		_interpolatedWeights = std::move(interpolatedWeights);
		_psi = std::move(psi);
		_psiTri = std::move(psiTri);
		_psiQuad = std::move(psiQuad);
	}

	Eigen::MatrixXd _skinningMatrix;
	Eigen::MatrixXd _weights;
	Eigen::MatrixXd _interpolatedWeights;
	Eigen::MatrixXd _psi;
	std::vector<double> _psiTri;
	std::vector<Eigen::Vector4d> _psiQuad;
};

struct InterpolatedVertexData
{
	float _translationFactor;
	Eigen::MatrixXd _vertices;
};

struct MeshComputeDeformationOperationResult
{
	void Update(std::vector<InterpolatedVertexData> vertexData)
	{
		_vertexData = std::move(vertexData);
	}

	std::vector<InterpolatedVertexData> _vertexData;
};

struct ProjectData
{
	ProjectData(const DeformationType deformationType,
		const LBC::DataSetup::WeightingScheme LBCWeightingScheme,
		EigenMesh mesh,
		EigenMesh cage,
		EigenMesh deformedCage,
		Eigen::MatrixXi cagePoints,
		Eigen::MatrixXd normals,
		Eigen::VectorXi b,
		Eigen::MatrixXd bc,
		std::optional<Eigen::MatrixXd> weights,
		std::optional<Parametrization> parametrization,
		std::optional<EigenMesh> embedding,
#ifdef WITH_SOMIGLIANA
		const std::shared_ptr<green::somig_deformer_3>& somiglianaDeformer,
		const double somigNu,
#endif
		const int32_t modelVerticesOffset,
		const int32_t numBBWSteps,
		const int32_t numSamples,
		const float scalingFactor,
		const glm::vec3& centerOffset,
		const bool interpolateWeights,
		const bool findOffset,
		const bool noOffset)
		: _deformationType(deformationType)
		, _LBCWeightingScheme(LBCWeightingScheme)
		, _mesh(std::move(mesh))
		, _cage(std::move(cage))
		, _deformedCage(std::move(deformedCage))
		, _cagePoints(std::move(cagePoints))
		, _normals(std::move(normals))
		, _b(std::move(b))
		, _bc(std::move(bc))
		, _weights(std::move(weights))
		, _parametrization(std::move(parametrization))
		, _embedding(std::move(embedding))
#ifdef WITH_SOMIGLIANA
		, _somiglianaDeformer(somiglianaDeformer)
		, _somigNu(somigNu)
#endif
		, _modelVerticesOffset(modelVerticesOffset)
		, _numBBWSteps(numBBWSteps)
		, _numSamples(numSamples)
		, _scalingFactor(scalingFactor)
		, _centerOffset(centerOffset)
		, _interpolateWeights(interpolateWeights)
		, _findOffset(findOffset)
		, _noOffset(noOffset)
	{
	}

	ProjectData()
		: _interpolateWeights(false)
		, _findOffset(false)
		, _noOffset(false)
	{ }

	[[nodiscard]] bool HarmonicOrLBC() const
	{
		return _deformationType == DeformationType::LBC || _deformationType == DeformationType::Harmonic;
	}

	[[nodiscard]] bool CanInterpolateWeights() const
	{
		return _interpolateWeights && HarmonicOrLBC();
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

	EigenMesh _mesh;
	EigenMesh _cage;
	EigenMesh _deformedCage;

	Eigen::MatrixXi _cagePoints;
	Eigen::MatrixXd _normals;
	Eigen::VectorXi _b;
	Eigen::MatrixXd _bc;
	std::optional<Eigen::MatrixXd> _weights;

	std::optional<Parametrization> _parametrization;
	std::optional<EigenMesh> _embedding;

#ifdef WITH_SOMIGLIANA
	std::shared_ptr<green::somig_deformer_3> _somiglianaDeformer = nullptr;

	double _somigNu = 0;
#endif

	int32_t _modelVerticesOffset = 0;
	int32_t _numBBWSteps = 300;
	int32_t _numSamples = 1;

	/// Scaling based on the bounding box of the file, so we can scale it down to fit in the viewport during load.
	/// On export we scale it back up by the inverse to get the final result.
	float _scalingFactor = 1.0f;

	/// The offset of the object in model space to center it in the viewport. On export we have to multiply by the inverse.
	glm::vec3 _centerOffset { 0.0f };

	/// Interpolate the weights.
	uint32_t _interpolateWeights : 1;
	uint32_t _findOffset : 1;
	uint32_t _noOffset : 1;
};
