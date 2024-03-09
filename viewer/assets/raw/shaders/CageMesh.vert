#version 450

#include "Common.inc.glsl"

layout (set = 0, binding = 0) uniform FrameDataBlock
{
	PerFrameData FrameData;
};
layout (set = 1, binding = 2) uniform ObjectDataBlock
{
	PerObjectData ObjectData;
};

layout (location = 0) in vec3 InPosition;
layout (location = 1) in vec3 InNormal;
layout (location = 2) in vec3 InVertexColor;

layout (location = 0) out vec3 InOutVertexColor;

void main()
{
	InOutVertexColor = InVertexColor;

	gl_Position = FrameData.Projection * FrameData.View * ObjectData.Model * vec4(InPosition, 1.0);
}
