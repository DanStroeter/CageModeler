#version 460
#extension GL_EXT_ray_tracing : require

struct HitData {
    uint faceIndex;
    float barycentricU;
    float barycentricV;
    float distance;
    uint sourceVertex;
    uint rayIndex;
    uint padding[2];
};

layout(location = 0) rayPayloadInEXT HitData payload;

void main()
{
    // Leave payload.faceIndex as 0xFFFFFFFF to indicate miss
    // Other fields are ignored
}