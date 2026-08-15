#version 460
#extension GL_EXT_ray_tracing : require

layout(location = 0) rayPayloadInEXT uint payload;

void main()
{
    // Do nothing.
    // payload stays 0xFFFFFFFF (miss)
}