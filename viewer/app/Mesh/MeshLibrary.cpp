#include <Mesh/MeshLibrary.h>

#include <cagedeformations/LoadMesh.h>
#include <cagedeformations/LoadFBX.h>

#include <glm/ext/scalar_constants.hpp>

namespace
{
	std::array ArrowPoints = {
		glm::vec2 { 0.25f, 0.0f },
		glm::vec2 { 0.25f, 0.05f },
		glm::vec2 { 1.0f, 0.05f },
		glm::vec2 { 1.0f, 0.10f },
		glm::vec2 { 1.2f, 0.0f }
	};

	std::array RingPoints = {
		glm::vec2 { 0.025f, 1.0f },
		glm::vec2 { -0.025f, 1.0f },
		glm::vec2 { -0.025f, 1.0f },
		glm::vec2 { -0.025f, 1.1f },
		glm::vec2 { -0.025f, 1.1f },
		glm::vec2 { 0.025f, 1.1f },
		glm::vec2 { 0.025f, 1.1f },
		glm::vec2 { 0.025f, 1.0f }
	};

	std::array MacePoints = {
		glm::vec2 { 0.25f, 0.0f },
		glm::vec2 { 0.25f, 0.05f },
		glm::vec2 { 1.0f, 0.05f },
		glm::vec2 { 1.0f, 0.1f },
		glm::vec2 { 1.25f, 0.1f },
		glm::vec2 { 1.25f, 0.0f }
	};
}

MeshGeometry MeshLibrary::MakeBox(const glm::vec3& minb, const glm::vec3& maxb)
{
	MeshGeometry geom;

	geom._positions = {
		glm::vec3 { minb.x, minb.y, minb.z }, glm::vec3 { minb.x, minb.y, maxb.z },
		glm::vec3 { minb.x, maxb.y, maxb.z }, glm::vec3 { minb.x, maxb.y, minb.z },
		glm::vec3 { maxb.x, minb.y, minb.z }, glm::vec3 { maxb.x, maxb.y, minb.z },
		glm::vec3 { maxb.x, maxb.y, maxb.z }, glm::vec3 { maxb.x, minb.y, maxb.z },
		glm::vec3 { minb.x, minb.y, minb.z }, glm::vec3 { maxb.x, minb.y, minb.z },
		glm::vec3 { maxb.x, minb.y, maxb.z }, glm::vec3 { minb.x, minb.y, maxb.z },
		glm::vec3 { minb.x, maxb.y, minb.z }, glm::vec3 { minb.x, maxb.y, maxb.z },
		glm::vec3 { maxb.x, maxb.y, maxb.z }, glm::vec3 { maxb.x, maxb.y, minb.z },
		glm::vec3 { minb.x, minb.y, minb.z }, glm::vec3 { minb.x, maxb.y, minb.z },
		glm::vec3 { maxb.x, maxb.y, minb.z }, glm::vec3 { maxb.x, minb.y, minb.z },
		glm::vec3 { minb.x, minb.y, maxb.z }, glm::vec3 { maxb.x, minb.y, maxb.z },
		glm::vec3 { maxb.x, maxb.y, maxb.z }, glm::vec3 { minb.x, maxb.y, maxb.z }
	};
	geom._indices = {
		0, 1, 2, 0, 2, 3, 4, 5, 6 , 4, 6, 7, 8, 9, 10,
		8, 10, 11, 12, 13, 14, 12, 14, 15, 16, 17, 18,
		16, 18, 19, 20, 21, 22, 20, 22, 23
	};

	return geom;
}

MeshGeometry MeshLibrary::MakeCylinder(const glm::vec3& axis,
	const glm::vec3& arm1,
	const glm::vec3& arm2,
	const uint32_t numSegments)
{
	MeshGeometry geom;

	geom._positions.reserve(2 * numSegments);
	geom._colors.reserve(2 * numSegments);
	geom._indices.reserve(2 * numSegments);

	for (std::size_t i = 0; i <= numSegments; ++i)
	{
		const auto angle = static_cast<float>(i % numSegments) * 2.0f * glm::pi<float>() / numSegments;
		const auto arm = arm1 * std::cos(angle) + arm2 * std::sin(angle);
		geom._positions.push_back(arm);
		geom._positions.push_back(arm + axis);
	}
	for (std::size_t i = 0; i < numSegments; ++i)
	{
		geom._indices.push_back(i * 2);
		geom._indices.push_back(i * 2 + 2);
		geom._indices.push_back(i * 2 + 3);
		geom._indices.push_back(i * 2);
		geom._indices.push_back(i * 2 + 3);
		geom._indices.push_back(i * 2 + 1);
	}

	// Generate caps.
	uint32_t base = static_cast<uint32_t>(geom._positions.size());
	for (std::size_t i = 0; i < numSegments; ++i)
	{
		const auto angle = static_cast<float>(i % numSegments) * 2.0f * glm::pi<float>() / numSegments;
		const auto arm = arm1 * std::cos(angle) + arm2 * std::sin(angle);
		geom._positions.push_back(arm + axis);
		geom._positions.push_back(arm);
	}
	for (uint32_t i = 2; i < numSegments; ++i)
	{
		geom._indices.push_back(base);
		geom._indices.push_back(base + i * 2 - 2);
		geom._indices.push_back(base + i * 2);
		geom._indices.push_back(base + 1);
		geom._indices.push_back(base + i * 2 + 1);
		geom._indices.push_back(base + i * 2 - 1);
	}

	return geom;
}

MeshGeometry MeshLibrary::MakeLathe(const glm::vec3& axis,
	const glm::vec3& arm1,
	const glm::vec3& arm2,
	const uint32_t numSegments,
	const std::span<glm::vec2> points,
	const glm::vec3& color,
	const float eps)
{
	MeshGeometry geom;

	geom._positions.reserve(numSegments * points.size());
	geom._colors.reserve(numSegments * points.size());
	geom._indices.reserve(2 * numSegments * points.size());

	for (uint32_t i = 0; i <= numSegments; ++i)
	{
		const float angle = (static_cast<float>(i % numSegments) * 2.0f * glm::pi<float>() / numSegments) + (glm::pi<float>() / 4.0f);
		const auto c = std::cos(angle);
		const auto s = std::sin(angle);
		const glm::mat2x3 mat = { axis, arm1 * c + arm2 * s };

		for (const auto& p : points)
		{
			geom._positions.push_back(mat * p + eps);
			geom._colors.push_back(color);
		}

		if (i > 0)
		{
			const auto numPoints = static_cast<uint32_t>(points.size());
			for (std::size_t j = 1; j < numPoints; ++j)
			{
				const auto i0 = (i - 1) * numPoints + (j - 1);
				const auto i1 = i * numPoints + (j - 1);
				const auto i2 = i * numPoints + j;
				const auto i3 = (i - 1) * numPoints + j;
				geom._indices.push_back(i0);
				geom._indices.push_back(i1);
				geom._indices.push_back(i2);
				geom._indices.push_back(i0);
				geom._indices.push_back(i2);
				geom._indices.push_back(i3);
			}
		}
	}

	return geom;
}

MeshGeometry MeshLibrary::MakeTranslate(const GizmoInteractionAxis axis)
{
	if (axis == GizmoInteractionAxis::X)
	{
		return MakeLathe(glm::vec3(1, 0, 0),
			glm::vec3(0, 1, 0),
			glm::vec3(0, 0, 1),
			16,
			ArrowPoints,
			glm::vec3(1.0f, 0.0f, 0.0f));
	}
	else if (axis == GizmoInteractionAxis::Y)
	{
		return MakeLathe(glm::vec3(0, 1, 0),
			glm::vec3(0, 0, 1),
			glm::vec3(1, 0, 0),
			16,
			ArrowPoints,
			glm::vec3(0.0f, 1.0f, 0.0f));
	}
	else
	{
		return MakeLathe(glm::vec3(0, 0, 1),
			glm::vec3(1, 0, 0),
			glm::vec3(0, 1, 0),
			16,
			ArrowPoints,
			glm::vec3(0.0f, 0.0f, 1.0f));
	}
}

MeshGeometry MeshLibrary::MakeRotation(const GizmoInteractionAxis axis)
{
	if (axis == GizmoInteractionAxis::X)
	{
		return MakeLathe(glm::vec3(1, 0, 0),
			glm::vec3(0, 1, 0),
			glm::vec3(0, 0, 1),
			32,
			RingPoints,
			glm::vec3(1.0f, 0.0f, 0.0f),
			0.003f);
	}
	else if (axis == GizmoInteractionAxis::Y)
	{
		return MakeLathe(glm::vec3(0, 1, 0),
			glm::vec3(0, 0, 1),
			glm::vec3(1, 0, 0),
			32,
			RingPoints,
			glm::vec3(0.0f, 1.0f, 0.0f),
			-0.003f);
	}
	else
	{
		return MakeLathe(glm::vec3(0, 0, 1),
			glm::vec3(1, 0, 0),
			glm::vec3(0, 1, 0),
			32,
			RingPoints,
			glm::vec3(0.0f, 0.0f, 1.0f));
	}
}

MeshGeometry MeshLibrary::MakeScale(const GizmoInteractionAxis axis)
{
	if (axis == GizmoInteractionAxis::X)
	{
		return MakeLathe(glm::vec3(1, 0, 0),
			glm::vec3(0, 1, 0),
			glm::vec3(0, 0, 1),
			16,
			MacePoints,
			glm::vec3(1.0f, 0.0f, 0.0f),
			0.003f);
	}
	else if (axis == GizmoInteractionAxis::Y)
	{
		return MakeLathe(glm::vec3(0, 1, 0),
			glm::vec3(0, 0, 1),
			glm::vec3(1, 0, 0),
			16,
			MacePoints,
			glm::vec3(0.0f, 1.0f, 0.0f),
			-0.003f);
	}
	else
	{
		return MakeLathe(glm::vec3(0, 0, 1),
			glm::vec3(1, 0, 0),
			glm::vec3(0, 1, 0),
			16,
			MacePoints,
			glm::vec3(0.0f, 0.0f, 1.0f));
	}
}

std::optional<EigenMesh> MeshLibrary::LoadMesh(const std::filesystem::path& filepath, const float scale)
{
	// if (filepath.extension() == ".fbx")
	// {
	// 	if (!load_fbx_file(filepath, vertices, indices, scale))
	// 	{
	// 		LOG_ERROR("Could not load mesh at filepath {}.", filepath.string().c_str());
	//
	// 		return;
	// 	}
	// }
	// else if (filepath.extension() == ".obj")
	// {

	// }
	// else if (filepath.extension() == ".msh")
	// {
	//
	// }

	Eigen::MatrixXd vertices;
	Eigen::MatrixXi indices;
	if (!load_mesh(filepath.string(), vertices, indices, scale))
	{
		LOG_ERROR("Could not load mesh filepath {}.", filepath.string().c_str());

		return { };
	}

	vertices.transposeInPlace();
	indices.transposeInPlace();

	return EigenMesh { std::move(vertices), std::move(indices) };
}
