#version 450

#include "Common.inc.glsl"

layout (location = 0) out vec4 OutColor;

layout (location = 0) in vec3 InOutPosition;
layout (location = 1) in vec3 InOutNormal;
layout (location = 2) in vec3 InOutVertexColor;
layout (location = 3) in vec3 InOutEyePosition;

void main()
{
	// Force the gizmo to always show up on top of all other objects.
	gl_FragDepth = 0.01;

	vec3 Color = max(dot(normalize(InOutNormal), normalize(InOutEyePosition - InOutPosition)), 0.5) + vec3(0.25);

	OutColor = vec4(LinearToSRGB(InOutVertexColor * Color), 1.0);
}