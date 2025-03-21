#version 450

#include "Common.inc.glsl"

layout (location = 0) out vec4 OutColor;

const vec3 StartGradientColor = vec3(0.125, 0.145, 0.14);
const vec3 EndGradientColor = vec3(0.09, 0.12, 0.115);
const vec2 GradientOffset = vec2(0.0, -0.25);

layout (set = 0, binding = 0) uniform FrameDataBlock
{
	PerFrameData FrameData;
};

vec4 ComputeNormalizedDeviceCoordinates()
{
	vec4 NormDeviceCoordinates = vec4(
		(gl_FragCoord.x / FrameData.ViewportSize.x - 0.5) * 2.0,
		(gl_FragCoord.y / FrameData.ViewportSize.y - 0.5) * 2.0,
		(gl_FragCoord.z - 0.5) * 2.0,
		1.0);

	return NormDeviceCoordinates;
}

vec3 Blend(vec3 source, vec3 dest, float alpha)
{
	return source * alpha + dest * (1.0 - alpha);
}

void main()
{
	vec4 NormDeviceCoordinates = ComputeNormalizedDeviceCoordinates();
	float Distance = length(NormDeviceCoordinates.xy / NormDeviceCoordinates.w - GradientOffset);
	vec3 Color = Blend(StartGradientColor, EndGradientColor, 1.0 - Distance * Distance);

	OutColor = vec4(LinearToSRGB(Color), 1.0);
}