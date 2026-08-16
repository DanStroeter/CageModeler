#pragma once

#include <Mesh/Operations/MeshOperation.h>
#include <Mesh/Operations/MeshWeightsParams.h>

#include <cagedeformations/InfluenceMap.h>

#include <filesystem>
#include <optional>
#include <vector>
#include <cstdint>

struct MeshExportDistanceFieldOperationParams
{
	MeshExportDistanceFieldOperationParams(EigenMesh mesh,
		EigenMesh cage,
		Parametrization parametrization,
		std::optional<std::vector<int32_t>> selectedVertices,
		std::filesystem::path outputFilename,
		const Eigen::MatrixXf* interiorDetours,
		DistanceColorMapParams colorMapParams)
		: _mesh(std::move(mesh))
		, _cage(std::move(cage))
		, _parametrization(std::move(parametrization))
		, _selectedVertices(std::move(selectedVertices))
		, _outputFilepath(std::move(outputFilename))
		, _interiorDetours(interiorDetours)
		, _colorMapParams(std::move(colorMapParams))
	{ }

	EigenMesh _mesh;
	EigenMesh _cage;
	Parametrization _parametrization { };

	/// The cage vertex the distances are measured from, specified exactly like the one of
	/// the influence color map. Only the first selected vertex is used.
	std::optional<std::vector<int32_t>> _selectedVertices;

	std::filesystem::path _outputFilepath;

	/// The already computed interior detour table (rows = cage vertices, cols = mesh
	/// vertices), owned by the caller. A null table exports the Euclidean distance field
	/// instead, which is the debugging counterpart of the interior one.
	const Eigen::MatrixXf* _interiorDetours = nullptr;

	DistanceColorMapParams _colorMapParams;
};

/**
 * Operation that will export the interior (or Euclidean) distance field of the mesh to a
 * selected cage vertex into a file, in the same vertex colored OBJ format as the influence
 * color map.
 */
class MeshExportDistanceFieldOperation final : public MeshOperationTemplated<MeshExportDistanceFieldOperationParams, void>
{
public:
	using MeshOperationTemplated::MeshOperationTemplated;

	[[nodiscard]] std::string GetDescription() const override
	{
		return (_params._interiorDetours != nullptr)
			? "Exporting interior distance color map"
			: "Exporting euclidean distance color map";
	}

	void Execute();
};
