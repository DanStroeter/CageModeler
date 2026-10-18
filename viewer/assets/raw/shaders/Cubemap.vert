#version 450

layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
    int faceIndex;
} pc;

// Vertex attributes
layout(location = 0) in vec3 InPosition;
layout(location = 1) in uint InTriangleID;
layout(location = 2) in uint InVertexIndex; // 0,1,2 per triangle

// Outputs to fragment shader
layout(location = 0) out vec3 OutColor;
layout(location = 1) flat out float TriangleID;

layout(set = 0, binding = 0) uniform FrameDataBlock
{
    float invNumTriangles;
} ubo;

void main()
{
    // Assign color based on vertex index
    vec3 color;
    if (InVertexIndex == 0) color = vec3(1.0, 0.0, 0.0); // red
    else if (InVertexIndex == 1) color = vec3(0.0, 1.0, 0.0); // green
    else color = vec3(0.0, 0.0, 1.0); // blue

    OutColor = color;
    TriangleID = float(InTriangleID) * ubo.invNumTriangles;

    gl_Position = pc.proj * pc.view * vec4(InPosition, 1.0);
}