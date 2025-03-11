#pragma once

#include <Mesh/Operations/MeshOperation.h>
#include <Mesh/Operations/MeshWeightsParams.h>

struct MeshComputeWeightsOperationParams
{
	MeshComputeWeightsOperationParams(const DeformationType deformationType,
		const LBC::DataSetup::WeightingScheme LBCScheme,
		EigenMesh mesh,
		EigenMesh cageMesh,
		std::optional<EigenMesh> embedding,
		std::optional<Eigen::MatrixXd> weights,
		const std::shared_ptr<somig_deformer_3>& somiglianaDeformer,
		Eigen::MatrixXi cagePoints,
		Eigen::MatrixXd normals,
		Eigen::VectorXi b,
		Eigen::MatrixXd bc,
		const bool interpolateWeights,
		const bool forceCalculateWeights,
		const int32_t numBBWSteps = 300,
		const int32_t numSamples = 1)
		: _deformationType(deformationType)
		, _LBCScheme(LBCScheme)
		, _mesh(std::move(mesh))
		, _cage(std::move(cageMesh))
		, _embedding(std::move(embedding))
		, _weights(std::move(weights))
		, _somiglianaDeformer(somiglianaDeformer)
		, _cagePoints(std::move(cagePoints))
		, _normals(std::move(normals))
		, _b(std::move(b))
		, _bc(std::move(bc))
		, _interpolateWeights(interpolateWeights)
		, _forceCalculateWeights(forceCalculateWeights)
		, _numBBWSteps(numBBWSteps)
		, _numSamples(numSamples)
	{ }

	DeformationType _deformationType = DeformationType::Green;
	LBC::DataSetup::WeightingScheme _LBCScheme = LBC::DataSetup::WeightingScheme::SQUARE;
	EigenMesh _mesh;
	EigenMesh _cage;
	std::optional<EigenMesh> _embedding;
	std::optional<Eigen::MatrixXd> _weights;
	std::shared_ptr<somig_deformer_3> _somiglianaDeformer = nullptr;

	Eigen::MatrixXi _cagePoints;
	Eigen::MatrixXd _normals;
	Eigen::VectorXi _b;
	Eigen::MatrixXd _bc;
	uint32_t _interpolateWeights : 1;
	uint32_t _forceCalculateWeights : 1;
	int32_t _numBBWSteps = 300;
	int32_t _numSamples = 2;
};

class MeshComputeWeightsOperation final : public MeshOperationTemplated<MeshComputeWeightsOperationParams, MeshComputeWeightsOperationResult>
{
public:
	using MeshOperationTemplated::MeshOperationTemplated;

	[[nodiscard]] std::string GetDescription() const override
	{
		return "Computing weights";
	}

	ExecutionResult Execute();
};
