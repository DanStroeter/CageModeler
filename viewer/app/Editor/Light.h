#pragma once

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

struct PointLight
{
	PointLight() = default;
	PointLight(const glm::vec3& position, const float intensity)
		: _position(position)
		, _intensity(intensity)
	{ }

	glm::vec3 _position { 0.0f };
	float _intensity = 1.0f;
};

struct PointLightGPU
{
	PointLightGPU() = default;
	PointLightGPU(const glm::vec3& position, const float intensity)
		: _positionAndIntensity(glm::vec4(position.x, position.y, position.z, intensity))
	{ }

	ALIGN_SIZE(16) glm::vec4 _positionAndIntensity { 0.0f };
};
