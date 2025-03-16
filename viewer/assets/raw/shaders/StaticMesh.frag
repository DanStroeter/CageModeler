#version 450

#include "Common.inc.glsl"

const int MAX_NUMBER_OF_LIGHTS = 8;

layout (constant_id = 0) const int RENDER_MODE = 0;

layout (set = 1, binding = 1) uniform LightUniform {
	LightInfo Lights[MAX_NUMBER_OF_LIGHTS];
};

const MaterialInfo Material = MaterialInfo(vec3(0.07, 0.07, 0.07), vec3(0.03, 0.03, 0.03), vec3(0.01, 0.01, 0.01), 32.0);

layout (location = 0) out vec4 OutColor;

layout (location = 0) in vec3 InOutPosition;
layout (location = 1) in vec3 InOutNormal;
layout (location = 2) in vec3 InOutVertexColor;

vec3 BlinnPhong(vec3 Position, vec3 Normal, int LightIndex)
{
	vec3 PositionToLightDir = normalize(vec3(Lights[LightIndex].PositionAndIntensity.xyz) - Position);
	vec3 Reflection = reflect(-PositionToLightDir, Normal);

	vec3 Ambient = Lights[LightIndex].PositionAndIntensity.w * Material.AmbientReflectivity;
	float PositionDotNormal = max(dot(PositionToLightDir, Normal), 0.0);
	vec3 Diffuse = Lights[LightIndex].PositionAndIntensity.w * Material.DiffuseReflectivity * PositionDotNormal;
	vec3 Specular = vec3(0.0);

	if (PositionDotNormal > 0.0)
	{
		float HighlightValue = max(dot(Reflection, Normal), 0.0);
		Specular = Lights[LightIndex].PositionAndIntensity.w * Material.SpecularReflectivity * pow(HighlightValue, Material.Shininess);
	}

	return Ambient + Diffuse + Specular;
}

void main()
{
	vec3 Color = vec3(0.0);
	for (int LightIndex = 0; LightIndex < MAX_NUMBER_OF_LIGHTS; LightIndex++)
	{
		Color += BlinnPhong(InOutPosition, normalize(InOutNormal), LightIndex);
	}

	if (RENDER_MODE == 0)
	{
		OutColor = vec4(LinearToSRGB(Color), 1.0);
	}
	else if (RENDER_MODE == 1)
	{
		OutColor = vec4(LinearToSRGB(InOutVertexColor), 1.0);
	}
}