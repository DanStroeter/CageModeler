#version 450

layout(location = 0) in vec3 inPosition;   // vertex position
layout(location = 1) in uint inTriangleID; // triangle index for this vertex

layout(location = 0) out vec3 fragColor;
layout(location = 1) flat out float fragTriangleID; // flat: not interpolated

layout(set = 0, binding = 0) uniform Matrices {
    mat4 uMVP;
} matrices;

void main()
{
    gl_Position = matrices.uMVP * vec4(inPosition, 1.0);

    // Determine color based on vertex index within triangle
    uint vertexIndex = gl_VertexIndex % 3; // 0,1,2 for each triangle
    if (vertexIndex == 0) fragColor = vec3(1.0, 0.0, 0.0); // red
    else if (vertexIndex == 1) fragColor = vec3(0.0, 1.0, 0.0); // green
    else fragColor = vec3(0.0, 0.0, 1.0); // blue

    fragTriangleID = float(inTriangleID); // store triangle index
}