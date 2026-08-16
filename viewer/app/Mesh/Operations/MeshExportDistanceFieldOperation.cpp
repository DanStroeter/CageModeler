#include <Mesh/Operations/MeshExportDistanceFieldOperation.h>

#include <Logging/Logging.h>

#include <string>

void MeshExportDistanceFieldOperation::Execute()
{
	// The source point is selected exactly like the control vertices of the influence map.
	const auto controlVerticesIdx = ResolveControlVertexIndices(_params._selectedVertices, _params._parametrization);
	if (controlVerticesIdx.empty())
	{
		LOG_WARN("Skipping the distance field export because no cage vertex was selected.");

		return;
	}

	const auto sourceVertexIdx = controlVerticesIdx.front();
	if (controlVerticesIdx.size() > 1)
	{
		LOG_WARN("The distance field is measured from a single point, using the first of the {} selected cage vertices ({}).",
			controlVerticesIdx.size(),
			sourceVertexIdx);
	}

	const auto& meshVertices = _params._mesh._vertices;
	const auto& cageVertices = _params._cage._vertices;
	if (sourceVertexIdx < 0 || sourceVertexIdx >= cageVertices.rows())
	{
		LOG_WARN("Skipping the distance field export because the selected cage vertex {} is out of range ({} cage vertices).",
			sourceVertexIdx,
			cageVertices.rows());

		return;
	}

	const auto* interiorDetours = _params._interiorDetours;
	const auto isInterior = (interiorDetours != nullptr);
	if (isInterior && (interiorDetours->rows() != cageVertices.rows() || interiorDetours->cols() != meshVertices.rows()))
	{
		LOG_WARN("Skipping the interior distance field export because the interior distance table ({}x{}) does not match the current project ({} cage x {} mesh vertices).",
			interiorDetours->rows(),
			interiorDetours->cols(),
			cageVertices.rows(),
			meshVertices.rows());

		return;
	}

	const Eigen::Vector3d sourcePosition = cageVertices.row(sourceVertexIdx).leftCols<3>();

	Eigen::VectorXd distances(meshVertices.rows());
	for (Eigen::Index i = 0; i < meshVertices.rows(); ++i)
	{
		const Eigen::Vector3d position = meshVertices.row(i).leftCols<3>();
		const auto euclideanDistance = (position - sourcePosition).norm();

		// The precomputed table stores the detours (interior distance minus Euclidean
		// distance) the PMVC compute shader adds on top of its rasterized hit distance,
		// so the interior distance is read back the same way here instead of being
		// recomputed.
		distances(i) = isInterior
			? euclideanDistance + static_cast<double>((*interiorDetours)(sourceVertexIdx, i))
			: euclideanDistance;
	}

	auto colorMapParams = _params._colorMapParams;
	colorMapParams.label = std::string(isInterior ? "interior" : "euclidean") + " distance from cage vertex " + std::to_string(sourceVertexIdx);

	LOG_INFO("Exporting the {} distance field from cage vertex {} to '{}'.",
		isInterior ? "interior" : "euclidean",
		sourceVertexIdx,
		_params._outputFilepath.string());

	write_distance_color_map_OBJ(_params._outputFilepath.string(),
		meshVertices,
		_params._mesh._faces,
		distances,
		colorMapParams);
}
