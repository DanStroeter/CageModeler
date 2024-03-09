#pragma once

#include <Mesh/GeometryUtils.h>

struct ModelInfo
{
	ALIGN_SIZE(16) glm::mat4 _model { 1.0f };
	ALIGN_SIZE(16) glm::mat3 _normalMatrix { 0.0f };
};

struct FrameInfo
{
	ALIGN_SIZE(16) glm::mat4 _view;
	ALIGN_SIZE(16) glm::mat4 _projection;
	ALIGN_SIZE(8) glm::vec2 _viewportSize;
};

struct ViewInfo
{
	[[nodiscard]] glm::vec2 GetNDC(const glm::vec2& screenSpaceCoord) const
	{
		return glm::vec2(2.0f * screenSpaceCoord / _windowSize - 1.0f);
	}

	[[nodiscard]] glm::vec3 DeprojectScreenToWorld(const glm::vec2 screenSpaceCoord) const
	{
		const auto normalizedCoord = GetNDC(screenSpaceCoord);
		const auto screenPosition = glm::vec4(normalizedCoord.x, normalizedCoord.y, -1.0f, 1.0f);
		const auto worldPosition = _inverseViewProjection * screenPosition;

		return glm::vec3(worldPosition / worldPosition.w);
	}

	[[nodiscard]] glm::vec3 ProjectWorldToCamera(const glm::vec3 worldSpaceCoord) const
	{
		const auto cameraPosition = _projection * _view * glm::vec4(worldSpaceCoord, 1.0f);

		return glm::vec3(cameraPosition / cameraPosition.w);
	}

	[[nodiscard]] glm::vec3 ProjectCameraToScreen(const glm::vec3 cameraCoord) const
	{
		return (cameraCoord + glm::vec3(1.0f)) / 2.0f * glm::vec3(_windowSize.x, _windowSize.y, 1.0f);
	}

	[[nodiscard]] glm::vec3 ProjectWorldToScreen(const glm::vec3 worldSpaceCoord) const
	{
		const auto cameraPosition = _projection * _view * glm::vec4(worldSpaceCoord, 1.0f);

		return (glm::vec3(cameraPosition / cameraPosition.w) + glm::vec3(1.0f)) / 2.0f * glm::vec3(_windowSize.x, _windowSize.y, 1.0f);
	}

	[[nodiscard]] Ray DeprojectScreenToWorldRay(const glm::vec2 screenSpaceCoord) const
	{
		const auto normalizedCoord = GetNDC(screenSpaceCoord);
		const auto rayStartWorldPosition = _inverseViewProjection * glm::vec4(normalizedCoord.x, normalizedCoord.y, -1.0f, 1.0f);
		const auto rayStart = glm::vec3(rayStartWorldPosition / rayStartWorldPosition.w);
		const auto rayEndWorldPosition = _inverseViewProjection * glm::vec4(normalizedCoord.x, normalizedCoord.y, 1.0f, 1.0f);
		const auto rayEndPosition = glm::vec3(rayEndWorldPosition / rayEndWorldPosition.w);
		const auto rayDirWS = glm::normalize(rayEndPosition - rayStart);

		return Ray { rayStart, rayDirWS };
	}

	/// Camera view matrix.
	glm::mat4 _view;

	/// Inverse view projection to extract camera vectors.
	glm::mat4 _inverseView;

	/// Keep a copy of the inverse view projection matrix to avoid re-computing for screen to world deprojection.
	glm::mat4 _inverseViewProjection;

	/// The projection matrix of the camera.
	glm::mat4 _projection;

	/// The position of the camera.
	glm::vec3 _position;

	/// The screen extents.
	glm::vec2 _renderSize;

	/// The window size.
	glm::vec2 _windowSize;
};