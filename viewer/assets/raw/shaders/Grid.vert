#version 450

#include "Common.inc.glsl"

vec3 GridPlanePositions[6] = vec3[](
	vec3(1.0, 1.0, 0.0), vec3(-1.0, -1.0, 0.0), vec3(-1.0, 1.0, 0.0),
	vec3(-1.0, -1.0, 0.0), vec3(1.0, 1.0, 0.0), vec3(1.0, -1.0, 0.0)
);

layout (set = 0, binding = 0) uniform FrameDataBlock
{
	PerFrameData FrameData;
};

layout (location = 1) out vec3 InOutNearPoint;
layout (location = 2) out vec3 InOutFarPoint;
layout (location = 3) out mat4 InOutFragView;
layout (location = 7) out mat4 InOutFragProjection;

vec3 UnprojectPoint(vec3 Point)
{
	mat4 InvView = inverse(FrameData.View);
	mat4 InvProjection = inverse(FrameData.Projection);
	vec4 UnprojectedPoint = InvView * InvProjection * vec4(Point, 1.0);

	return UnprojectedPoint.xyz / UnprojectedPoint.w;
}

void main()
{
	vec3 GridPosition = GridPlanePositions[gl_VertexIndex].xyz;
	InOutNearPoint = UnprojectPoint(vec3(GridPosition.x, GridPosition.y, 0.0));
	InOutFarPoint = UnprojectPoint(vec3(GridPosition.x, GridPosition.y, 1.0));
	InOutFragView = FrameData.View;
	InOutFragProjection = FrameData.Projection;

	gl_Position = vec4(GridPosition, 1.0);
}
