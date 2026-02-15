#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_buffer_reference2 : require

hitAttributeEXT vec2 hitAttributes;

// Structs must match C++ exactly
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

// Acceleration structure (needed for recursive tracing)
layout(binding = 0, set = 0) uniform accelerationStructureEXT cageAccel;

// Hit buffer (binding 3)
layout(binding = 3, set = 0) buffer HitBuffer {
    HitRecord hits[];
} hitBuffer;

// Cage indices buffer (binding 4)
layout(binding = 4, set = 0) readonly buffer CageIndexBuffer {
    uint indices[];
} cageIndices;

layout(push_constant) uniform PushConstants {
    uint maxHitsPerRay;
    uint skipEveryNthHit;
    uint vertexCount;
    uint raysPerVertex;
} pc;

void main() {
    // Calculate hit index in the buffer
    uint hitIndex = payload.rayIndex * pc.maxHitsPerRay + payload.hitCount;
    
    // Get triangle info
    uint triIdx = gl_PrimitiveID;
    uint baseIdx = triIdx * 3;
    
    // Write hit record
    hitBuffer.hits[hitIndex].position = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * gl_HitTEXT;
    hitBuffer.hits[hitIndex].normal = gl_WorldRayDirectionEXT;  // Note: This is ray direction, not surface normal
    hitBuffer.hits[hitIndex].barycentric = hitAttributes;
    hitBuffer.hits[hitIndex].triangleId = triIdx;
    hitBuffer.hits[hitIndex].vertexIndices[0] = cageIndices.indices[baseIdx];
    hitBuffer.hits[hitIndex].vertexIndices[1] = cageIndices.indices[baseIdx + 1];
    hitBuffer.hits[hitIndex].vertexIndices[2] = cageIndices.indices[baseIdx + 2];
    hitBuffer.hits[hitIndex].distance = gl_HitTEXT;
    hitBuffer.hits[hitIndex].rayIndex = payload.rayIndex;
    hitBuffer.hits[hitIndex].hitSequence = payload.hitCount;
    
    // Increment hit count
    payload.hitCount++;
    
    // If we haven't reached max hits, continue tracing
    if (payload.hitCount < pc.maxHitsPerRay) {
        // Calculate new origin just past the current hit point
        // Add small epsilon to avoid self-intersection
        vec3 newOrigin = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * (gl_HitTEXT + 0.001);
        
        // Continue tracing from just past the hit point
        traceRayEXT(
            cageAccel,                      // acceleration structure
            gl_RayFlagsOpaqueEXT,           // ray flags
            0xFF,                           // cull mask
            0,                              // sbt record offset
            0,                              // sbt record stride
            0,                              // miss index
            newOrigin,                      // origin (past the hit)
            0.001,                          // tMin
            gl_WorldRayDirectionEXT,        // direction (same direction)
            payload.tMax,                   // tMax (remaining distance)
            0                               // payload location
        );
    }
}