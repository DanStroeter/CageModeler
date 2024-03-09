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

layout (location = 0) out vec3 InOutPosition;
layout (location = 1) out vec3 InOutNormal;
layout (location = 2) out vec3 InOutVertexColor;
layout (location = 3) out vec3 InOutEyePosition;

void main()
{
	InOutPosition = vec3(FrameData.View * ObjectData.Model * vec4(InPosition, 1.0));
	InOutNormal = normalize(ObjectData.NormalMatrix * InNormal);
	InOutVertexColor = InVertexColor;

	mat4 invView = inverse(FrameData.View);
	InOutEyePosition = vec3(invView[3][0], invView[3][1], invView[3][2]);

	gl_Position = FrameData.Projection * vec4(InOutPosition, 1.0);
}