#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : enable

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

hitAttributeEXT vec2 barycentrics;

void main()
{
    // Get hit information
    payload.faceIndex = gl_PrimitiveID;
    payload.barycentricU = barycentrics.x;
    payload.barycentricV = barycentrics.y;
    payload.distance = gl_HitTEXT;
    
    // Note: sourceVertex and rayIndex were already set in raygen
}