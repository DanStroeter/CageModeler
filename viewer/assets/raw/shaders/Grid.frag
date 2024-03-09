#version 450

#include "Common.inc.glsl"

layout (location = 0) out vec4 OutColor;

layout (location = 1) in vec3 InOutNearPoint;
layout (location = 2) in vec3 InOutFarPoint;
layout (location = 3) in mat4 InOutFragView;
layout (location = 7) in mat4 InOutFragProjection;

const float NearPlaneDistance = 0.01;
const float FarPlaneDistance = 10.0;

vec4 ComputeGridColor(vec3 FragmentPosition, float Scale)
{
	vec2 Coordinates = FragmentPosition.xz * Scale;
	vec2 Derivative = fwidth(Coordinates);
	vec2 GridPosition = abs(fract(Coordinates - 0.5) - 0.5) / Derivative;
	float Line = min(GridPosition.x, GridPosition.y);
	float MinZ = min(Derivative.y, 1);
	float MinX = min(Derivative.x, 1);
	vec4 Color = vec4(0.5, 0.5, 0.5, 1.0 - min(Line, 1.0));

	if (abs(FragmentPosition.x) < MinX)
	{
		Color.z = 1.0;
	}

	if (abs(FragmentPosition.z) < MinZ)
	{
		Color.x = 1.0;
	}

	return Color;
}

float ComputeDepth(vec3 Position)
{
	vec4 PositionClipSpace = InOutFragProjection * InOutFragView * vec4(Position.xyz, 1.0);

	return (PositionClipSpace.z / PositionClipSpace.w);
}

float ComputeLinearDepth(vec3 Position)
{
	vec4 PositionClipSpace = InOutFragProjection * InOutFragView * vec4(Position.xyz, 1.0);

	// Put back in range -1 to 1.
	float DepthClipSpace = (PositionClipSpace.z / PositionClipSpace.w) * 2.0 - 1.0;

	// Get linear value between near and far depth.
	float LinearDepth = (2.0 * NearPlaneDistance * FarPlaneDistance) / (FarPlaneDistance + NearPlaneDistance - DepthClipSpace * (FarPlaneDistance - NearPlaneDistance));

	// Normalize the point.
	return LinearDepth / FarPlaneDistance;
}

void main()
{
	float LineT = -InOutNearPoint.y / (InOutFarPoint.y - InOutNearPoint.y);
	vec3 FragmentPositionWS = InOutNearPoint + LineT * (InOutFarPoint - InOutNearPoint);

	// Write out the depth of the fragment to avoid having the objects appear behind the grid.
	gl_FragDepth = ComputeDepth(FragmentPositionWS);

	float LinearDepth = ComputeLinearDepth(FragmentPositionWS);
	float FadeAmount = max(0.0, (0.5 - LinearDepth));

	vec4 ColorA = ComputeGridColor(FragmentPositionWS, 1);
	vec4 ColorB = ComputeGridColor(FragmentPositionWS, 0.25);

	vec4 Color = (ColorA + ColorB) * float(LineT > 0.0);
	Color.a *= FadeAmount;

	OutColor = vec4(LinearToSRGB(Color.xyz), Color.a);
}
