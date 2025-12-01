#version 450

struct PerFrameData {
    mat4 Projection;
    mat4 View;
};

struct PerObjectData {
    mat4 Model;
};

layout(set = 0, binding = 0) uniform FrameDataBlock
{
    PerFrameData FrameData;
};

layout(set = 1, binding = 2) uniform ObjectDataBlock
{
    PerObjectData ObjectData;
};

// Vertex attributes
layout(location = 0) in vec3 InPosition;
layout(location = 1) in uint InTriangleID;
layout(location = 2) in uint InVertexIndex; // 0,1,2 per triangle

// Outputs to fragment shader
layout(location = 0) out vec3 OutColor;
layout(location = 1) flat out uint TriangleID;

void main()
{
    // Assign color based on vertex index
    vec3 color;
    if (InVertexIndex == 0) color = vec3(1.0, 0.0, 0.0); // red
    else if (InVertexIndex == 1) color = vec3(0.0, 1.0, 0.0); // green
    else color = vec3(0.0, 0.0, 1.0); // blue

    OutColor = color;
    TriangleID = InTriangleID; // flat, not interpolated

    gl_Position = FrameData.Projection * FrameData.View * ObjectData.Model * vec4(InPosition, 1.0);
}