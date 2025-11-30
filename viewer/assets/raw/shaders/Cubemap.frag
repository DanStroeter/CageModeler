#version 450

layout(location = 0) in vec3 fragColor;
layout(location = 1) flat in float fragTriangleID; // not interpolated

layout(location = 0) out vec4 outColor;

void main()
{
    outColor = vec4(fragColor, fragTriangleID); // RGB = color, A = triangle ID
}