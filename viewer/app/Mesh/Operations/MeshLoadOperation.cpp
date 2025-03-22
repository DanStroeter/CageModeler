#include <Mesh/Operations/MeshLoadOperation.h>
#include <Mesh/GeometryUtils.h>

#include <cagedeformations/Parametrization.h>
#include <cagedeformations/LoadMesh.h>
#include <cagedeformations/LoadFBX.h>
#include <cagedeformations/GreenCoordinates.h>
#include <igl/boundary_conditions.h>
#include <igl/EPS.h>
#include <igl/readDMAT.h>

#include <fstream>

namespace
{
	constexpr auto MaximumModelBoundsSize = 5.0f;
}

MeshLoadOperation::ExecutionResult MeshLoadOperation::Execute()
{
	// Load the parameters from a file if we have one.
	std::optional<Parametrization> parametrization;
	if (_params._parametersFilepath.has_value())
	{
		parametrization = readParams(_params._parametersFilepath.value().string());

		LOG_DEBUG("Loaded parametrization file at {}.", _params._parametersFilepath.value().string());
	}

	EigenMesh mesh;
	EigenMesh cage;

	// Load the model from the file and optionally the cage if we are using an FBX input file.
	if (_params.IsFBX())
	{
		if (!load_fbx_file(_params._meshFilepath.string(),
			mesh._vertices,
			mesh._faces,
			cage._vertices,
			cage._faces))
		{
			LOG_ERROR("Failed to load fbx file.");

			return ExecutionResult("Failed to load fbx file.");
		}
	}
	else
	{
		if (!load_mesh(_params._meshFilepath.string(),
			mesh._vertices,
			mesh._faces,
			1.0))
		{
			LOG_ERROR("Failed to load mesh file.");

			return ExecutionResult("Failed to load mesh file.");
		}
	}

	auto somiglianaDeformer = std::make_shared<somig_deformer_3>(_params._somigNu);

	if (_params._deformationType == DeformationType::Somigliana)
	{
		std::vector<std::vector<unsigned int>> elements(mesh._faces.rows());
		std::vector<point3d> model_verts(mesh._vertices.rows());

		for (int i = 0; i < mesh._vertices.rows(); ++i) {
			model_verts[i] = { mesh._vertices(i, 0), mesh._vertices(i, 1), mesh._vertices(i, 2) };
		}
		for (int i = 0; i < mesh._faces.rows(); ++i) {
			elements[i] = { static_cast<unsigned int>(mesh._faces(i, 0)),
				static_cast<unsigned int>(mesh._faces(i, 1)), static_cast<unsigned int>(mesh._faces(i, 2)) };
			/*if (msh) {
				elements[i].push_back(static_cast<unsigned int>(T_model(i, 3)));
			}*/
		}

		somiglianaDeformer->set_mesh(elements, model_verts);
	}

	LOG_DEBUG("Loaded mesh {}.", _params._meshFilepath.string());

	CheckFormat((DeformationTypeHelpers::RequiresEmbedding(_params._deformationType) && _params._embeddingFilepath.has_value()) ||
		!DeformationTypeHelpers::RequiresEmbedding(_params._deformationType), "Coordinates require an embedding, but one was not provided.");

	// Load the embedding.
	const auto hasEmbedding = DeformationTypeHelpers::RequiresEmbedding(_params._deformationType) && _params._embeddingFilepath.has_value();
	std::optional<EigenMesh> embedding;

	if (hasEmbedding)
	{
		embedding = EigenMesh();

		if (!load_mesh(_params._embeddingFilepath->string(),
			embedding->_vertices,
			embedding->_faces,
			1.0))
		{
			return ExecutionResult("Failed to load mesh embedding.");
		}

		LOG_DEBUG("Loaded embedding {}.", _params._embeddingFilepath->string());
	}

	int32_t modelVerticesOffset = 0;

	// Finding model verts in embedding.
	if (DeformationTypeHelpers::RequiresEmbedding(_params._deformationType))
	{
		if (_params._findOffset && !_params.CanInterpolateWeights())
		{
			const auto vericesEqual = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b)
			{
				return abs(a(0) - b(0)) < igl::FLOAT_EPS &&
					abs(a(1) - b(1)) < igl::FLOAT_EPS &&
					abs(a(2) - b(2)) < igl::FLOAT_EPS;
			};

			const auto firstModelVert = mesh._vertices.row(0);
			bool found = false;

			for (int i = 0; i < embedding->_vertices.rows(); ++i)
			{
				const auto& embedding_vert = embedding->_vertices.row(i);

				if (vericesEqual(firstModelVert, embedding_vert))
				{
					found = true;
					modelVerticesOffset = i;

					break;
				}
			}

			if (found)
			{
				LOG_DEBUG("Found model verts in embedding with an offset of {}.", modelVerticesOffset);
			}
			else
			{
				LOG_ERROR("Could not find model verts in embedding.");

				return ExecutionResult("Could not find model verts in embedding.");
			}
		}
		else
		{
			const auto additionalOffset = _params.HasNoOffset() ? 0 : embedding->_vertices.rows() - (mesh._vertices.rows() + modelVerticesOffset);
			modelVerticesOffset += static_cast<int32_t>(additionalOffset);

			LOG_DEBUG("Adding an offset of {}.", additionalOffset);
		}
	}

	Eigen::VectorXi cagePoints;

	// Load the cage if we haven't loaded it from the FBX file first, so we can generate the parametrization in the next step.
	if (!_params.IsFBX())
	{
		const auto embeddingVertices = (DeformationTypeHelpers::RequiresEmbedding(_params._deformationType) ? &embedding->_vertices : nullptr);

		if (!load_cage(_params._cageFilepath.string(),
			cage._vertices,
			cagePoints,
			cage._faces,
			1.0,
			_params.ShouldTriangulateQuads(),
			embeddingVertices,
			_params._findOffset))
		{
			LOG_ERROR("Failed to load cage!.");

			return ExecutionResult("Failed to load cage.");
		}

		LOG_DEBUG("Loaded deformation mesh {}.", _params._cageFilepath.string());
	}

	if (_params._deformationType == DeformationType::Somigliana)
	{
		std::vector<std::vector<unsigned int>> faces_somig(cage._faces.rows());
		std::vector<point3d> verts_somig(cage._vertices.rows());
		for (int i = 0; i < cage._faces.rows(); ++i) {
			faces_somig[i] = { static_cast<unsigned int>(cage._faces(i, 0)), static_cast<unsigned int>(cage._faces(i, 1)),
				static_cast<unsigned int>(cage._faces(i, 2)) };
		}
		for (int i = 0; i < cage._vertices.rows(); ++i) {
			verts_somig[i] = { cage._vertices(i, 0), cage._vertices(i, 1), cage._vertices(i, 2) };
		}
		somiglianaDeformer->set_cage(faces_somig, verts_somig);

		// TODO: Fix Somigliana mesh scaling.
	}

	Eigen::MatrixXi BE;
	Eigen::MatrixXi CE;

	if (!_params._findOffset && !_params._interpolateWeights)
	{
		modelVerticesOffset = static_cast<int32_t>(cage._vertices.rows());
	}

	if (DeformationTypeHelpers::RequiresEmbedding(_params._deformationType))
	{
		LOG_DEBUG("Using {} as offset for model vertices in embedding.", modelVerticesOffset);
	}

	const auto deformedCageFilepath = (_params._deformedCageFilepath.has_value() ? _params._deformedCageFilepath.value() : _params._cageFilepath);

	// Construct the cage mesh from the input results in the window.
	EigenMesh deformedCage;
	Eigen::VectorXi cagePointsDeformed;

	if (!load_cage(deformedCageFilepath.string(),
		deformedCage._vertices,
		cagePointsDeformed,
		deformedCage._faces,
		1.0,
		_params.ShouldTriangulateQuads()))
	{
		LOG_ERROR("Failed to load deformed cage!.");

		return ExecutionResult("Failed to load deformed cage.");
	}

	if (!_params._parametersFilepath.has_value())
	{
		parametrization = fromDeformedCage(cage._vertices, deformedCage._vertices);
	}

	Eigen::MatrixXd normals;
	Eigen::VectorXi b;
	Eigen::MatrixXd bc;

	if (DeformationTypeHelpers::RequiresEmbedding(_params._deformationType))
	{
		LOG_DEBUG("Computing Boundary conditions.");

		if (!igl::boundary_conditions(embedding->_vertices,
			embedding->_faces,
			cage._vertices,
			cagePoints,
			BE,
			CE,
			cage._faces,
			b,
			bc))
		{
			LOG_ERROR("Failed to extract boundary conditions for cage!.");

			return ExecutionResult("Failed to extract boundary conditions for cage.");
		}

		LOG_DEBUG("Done computing boundary conditions.");
	}
	else if (_params._deformationType == DeformationType::Green)
	{
		calcNormals(cage._vertices, cage._faces, normals);
	}

	std::optional<Eigen::MatrixXd> weights { };

	if ((_params._deformationType == DeformationType::LBC || _params._deformationType == DeformationType::BBW) &&
		_params._weightsFilepath.has_value())
	{
		const auto weightsFilepath = _params._weightsFilepath->string();
		std::ifstream in(weightsFilepath);

		if (in.good())
		{
			LOG_DEBUG("Reading weights from file {}.", weightsFilepath);

			weights = Eigen::MatrixXd();
			if (!igl::readDMAT(weightsFilepath, weights.value()))
			{
				LOG_ERROR("Failed to read weights from file!.");

				return ExecutionResult("Failed to read weights from file.");
			}
		}
	}

	const auto meshAABB = GeometryUtils::ComputeAABB(mesh);
	const auto diff = meshAABB._max - meshAABB._min;
	const auto maxSide = _params._scalingFactor * std::max(std::max(diff.x, diff.y), diff.z);
	const auto internalScale = MaximumModelBoundsSize / maxSide;

	const auto aabbCenter = 0.5f * (meshAABB._min + meshAABB._max);
	const glm::vec3 centerOffset(-aabbCenter.x, -meshAABB._min.y, -aabbCenter.z);

	return std::make_shared<ProjectData>(_params._deformationType,
		_params._LBCWeightingScheme,
		std::move(mesh),
		std::move(cage),
		std::move(deformedCage),
		std::move(cagePoints),
		std::move(normals),
		std::move(b),
		std::move(bc),
		std::move(weights),
		std::move(parametrization),
		hasEmbedding ? std::move(embedding) : std::optional<EigenMesh>(),
		somiglianaDeformer,
		_params._somigNu,
		modelVerticesOffset,
		_params._numBBWSteps,
		_params._numSamples,
		internalScale,
		centerOffset,
		_params._interpolateWeights,
		_params._findOffset,
		_params._noOffset);
}
