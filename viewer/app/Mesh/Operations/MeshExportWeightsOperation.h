#pragma once

#include <Mesh/Operations/MeshOperation.h>
#include <Mesh/Operations/MeshWeightsParams.h>

#include <filesystem>
#include <utility>

#include <igl/writeDMAT.h>
#include <igl/bbw.h>

struct MeshExportWeightsOperationParams
{
	MeshExportWeightsOperationParams(const DeformationType deformationType,
		const LBC::DataSetup::WeightingScheme LBCScheme,
		std::filesystem::path outputFilename,
		Eigen::MatrixXd weights,
		Eigen::VectorXi b,
		Eigen::MatrixXd bc,
		EigenMesh embedding,
		const int32_t numBBWSteps)
		: _deformationType(deformationType)
		, _LBCScheme(LBCScheme)
		, _outputFilepath(std::move(outputFilename))
		, _weights(std::move(weights))
		, _b(std::move(b))
		, _bc(std::move(bc))
		, _embedding(std::move(embedding))
		, _numBBWSteps(numBBWSteps)
	{ }

	DeformationType _deformationType;
	LBC::DataSetup::WeightingScheme _LBCScheme;
	std::filesystem::path _outputFilepath;
	Eigen::MatrixXd _weights;
	Eigen::VectorXi _b;
	Eigen::MatrixXd _bc;
	EigenMesh _embedding;
	int32_t _numBBWSteps = 300;
};

/**
 * Operation that will export the computed weights into a file.
 */
class MeshExportWeightsOperation final : public MeshOperationTemplated<MeshExportWeightsOperationParams, void>
{
public:
	using MeshOperationTemplated::MeshOperationTemplated;

	[[nodiscard]] std::string GetDescription() const override
	{
		return "Exporting influence color map";
	}

	void Execute()
	{
		CheckFormat(_params._deformationType == DeformationType::LBC || _params._deformationType == DeformationType::BBW, "Cannot export params of type that is not LBC or BBW.");

		if (_params._deformationType == DeformationType::BBW)
		{
			// Only a few iterations for sake of demo.
			igl::BBWData bbwData;
			bbwData.active_set_params.max_iter = _params._numBBWSteps;
			bbwData.verbosity = 2;

			if (!igl::bbw(_params._embedding._vertices, _params._embedding._faces, _params._b, _params._bc, bbwData, _params._weights))
			{
				LOG_ERROR("Failed to compute bounded biharmonic weights!.");

				return;
			}
		}

		if (!igl::writeDMAT(_params._outputFilepath.string(), _params._weights))
		{
			LOG_ERROR("Failed to write weights to file!.");
		}
	}
};