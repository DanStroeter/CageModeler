#version 450

layout(location = 0) in vec3 OutColor;
layout(location = 1) flat in uint TriangleID;

layout(location = 0) out vec4 FragColor;

void main()
{
    // Triangle ID in alpha channel
    float id = float(TriangleID);

    // RGB comes from interpolated vertex colors
    FragColor = vec4(OutColor, id);
}
