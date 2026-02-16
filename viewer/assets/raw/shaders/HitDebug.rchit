#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : require

struct SimpleHit {
    uint  faceIndex;
    float barycentricU;
    float barycentricV;
    float distance;
    uint  sourceVertex;
    uint  rayIndex;
    uint  padding0;
    uint  padding1;
};

layout(set = 0, binding = 3, scalar) buffer HitBuffer {
    SimpleHit hits[];
};

layout(push_constant) uniform PushConstants {
    uint vertexCount;
    uint raysPerVertex;
    uint maxHitsPerRay;
} pc;

layout(location = 0) rayPayloadInEXT uint payload;

hitAttributeEXT vec2 attribs;

void main()
{
    uint rayIndex = payload;

    uint vertexIndex = rayIndex / pc.raysPerVertex;

    uint writeIndex = rayIndex;  // 1 hit per ray for debug

    hits[writeIndex].faceIndex    = gl_PrimitiveID;
    hits[writeIndex].barycentricU = attribs.x;
    hits[writeIndex].barycentricV = attribs.y;
    hits[writeIndex].distance     = gl_HitTEXT;
    hits[writeIndex].sourceVertex = vertexIndex;
    hits[writeIndex].rayIndex     = payload;
    hits[writeIndex].padding0     = 0;
    hits[writeIndex].padding1     = 0;
}