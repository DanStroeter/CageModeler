#pragma once

#include <Mesh/Operations/MeshOperation.h>
#include <Mesh/Operations/MeshWeightsParams.h>

#include <filesystem>
#include <utility>

class PolygonMesh;

enum class MeshExportType : uint8_t
{
	OBJ,
	FBX,
	MSH
};

struct DeformedMeshExportOperationParams
{
	DeformedMeshExportOperationParams(MeshComputeDeformationOperationResult deformationData,
		const DeformationType deformationType,
		const LBC::DataSetup::WeightingScheme LBCScheme,
		const std::optional<std::size_t> frameIndex,
#if WITH_SOMIGLIANA
		const std::shared_ptr<green::somig_deformer_3>& somiglianaDeformer,
#endif
		Eigen::MatrixXi faces,
		std::filesystem::path outputFilename,
		const float scalingFactor)
		: _deformationData(std::move(deformationData))
		, _deformationType(deformationType)
		, _LBCScheme(LBCScheme)
		, _frameIndex(frameIndex)
#if WITH_SOMIGLIANA
		, _somiglianaDeformer(somiglianaDeformer)
#endif
		, _faces(std::move(faces))
		, _outputFilename(std::move(outputFilename))
		, _scalingFactor(scalingFactor)
	{ }

	MeshComputeDeformationOperationResult _deformationData;
	DeformationType _deformationType;
	LBC::DataSetup::WeightingScheme _LBCScheme;
	std::optional<std::size_t> _frameIndex;
#if WITH_SOMIGLIANA
	std::shared_ptr<green::somig_deformer_3> _somiglianaDeformer;
#endif
	Eigen::MatrixXi _faces;
	std::filesystem::path _outputFilename;
	float _scalingFactor = 1.0f;
};

/**
 * Operation that will export the deformed mesh into a file.
 */
class DeformedMeshExportOperation final : public MeshOperationTemplated<DeformedMeshExportOperationParams, void>
{
public:
	using MeshOperationTemplated::MeshOperationTemplated;

	[[nodiscard]] std::string GetDescription() const override
	{
		return "Exporting deformed mesh";
	}

	void Execute();

private:
	void ExportSample(const std::size_t frameIndex, const std::string_view deformationTypeVariant);
};

struct MeshExportOperationParams
{
	MeshExportOperationParams(const DeformationType deformationType,
		const LBC::DataSetup::WeightingScheme LBCScheme,
		Eigen::MatrixXi faces,
		Eigen::MatrixXd vertices,
		std::filesystem::path outputFilename)
		: _deformationType(deformationType)
		, _LBCScheme(LBCScheme)
		, _faces(std::move(faces))
		, _vertices(std::move(vertices))
		, _outputFilename(std::move(outputFilename))
	{ }

	DeformationType _deformationType;
	LBC::DataSetup::WeightingScheme _LBCScheme;
	Eigen::MatrixXi _faces;
	Eigen::MatrixXd _vertices;
	std::filesystem::path _outputFilename;
};

/**
 * Operation that will export any mesh into a file.
 */
class MeshExportOperation final : public MeshOperationTemplated<MeshExportOperationParams, void>
{
public:
	using MeshOperationTemplated::MeshOperationTemplated;

	[[nodiscard]] std::string GetDescription() const override
	{
		return "Exporting mesh";
	}

	void Execute();
};