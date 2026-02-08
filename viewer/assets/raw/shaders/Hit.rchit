#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_buffer_reference2 : require

hitAttributeEXT vec2 hitAttributes;

// STRUCTS FIRST
struct RayPayload {
    uint vertexIndex;
    uint rayIndex;
    uint hitCount;
    float tMax;
};

struct HitRecord {
    vec3 position;
    vec3 normal;
    vec2 barycentric;
    uint triangleId;
    uint vertexIndices[3];
    float distance;
    uint rayIndex;
    uint hitSequence;
};

layout(location = 0) rayPayloadInEXT RayPayload payload;
layout(binding = 0, set = 0) uniform accelerationStructureEXT cageAccel;  // ADDED: acceleration structure
layout(binding = 4, set = 0) buffer HitBuffer {
    HitRecord hits[];
} hitBuffer;
layout(binding = 5, set = 0) readonly buffer CageIndexBuffer {
    uint indices[];
} cageIndices;

layout(push_constant) uniform PushConstants {
    uint maxHitsPerRay;
    uint skipEveryNthHit;
    uint vertexCount;
    uint raysPerVertex;
} pc;

void main() {
    uint hitIndex = payload.rayIndex * pc.maxHitsPerRay + payload.hitCount;
    uint triIdx = gl_PrimitiveID;
    uint baseIdx = triIdx * 3;
    
    hitBuffer.hits[hitIndex].position = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * gl_HitTEXT;
    hitBuffer.hits[hitIndex].normal = gl_WorldRayDirectionEXT;
    hitBuffer.hits[hitIndex].barycentric = hitAttributes;
    hitBuffer.hits[hitIndex].triangleId = triIdx;
    hitBuffer.hits[hitIndex].vertexIndices[0] = cageIndices.indices[baseIdx];
    hitBuffer.hits[hitIndex].vertexIndices[1] = cageIndices.indices[baseIdx + 1];
    hitBuffer.hits[hitIndex].vertexIndices[2] = cageIndices.indices[baseIdx + 2];
    hitBuffer.hits[hitIndex].distance = gl_HitTEXT;
    hitBuffer.hits[hitIndex].rayIndex = payload.rayIndex;
    hitBuffer.hits[hitIndex].hitSequence = payload.hitCount;
    
    payload.hitCount++;
    if (payload.hitCount < pc.maxHitsPerRay) {
        // Calculate new origin just past the current hit point
        vec3 newOrigin = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * (gl_HitTEXT + 0.001);
        
        // Continue tracing from just past the hit point
        traceRayEXT(
            cageAccel,                      // accelerationStructureEXT topLevel
            gl_RayFlagsOpaqueEXT,           // rayFlags
            0xFF,                           // cullMask
            0,                              // sbtRecordOffset
            0,                              // sbtRecordStride
            0,                              // missIndex
            newOrigin,                      // origin
            0.001,                          // Tmin (small epsilon to avoid self-intersection)
            gl_WorldRayDirectionEXT,        // direction
            payload.tMax,                   // Tmax
            0                               // payload (location = 0)
        );
    }
}