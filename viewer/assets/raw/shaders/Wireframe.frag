#version 450

#include "Common.inc.glsl"

layout (location = 0) out vec4 OutColor;

layout (location = 0) flat in vec3 InOutVertexColor;

void main()
{
	OutColor = vec4(LinearToSRGB(InOutVertexColor), 1.0);
}