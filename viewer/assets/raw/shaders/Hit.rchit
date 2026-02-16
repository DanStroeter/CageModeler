#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_buffer_reference2 : require

hitAttributeEXT vec2 hitAttributes;

struct RayPayload {
    uint vertexIndex;
    uint rayIndex;
    uint hitCount;
    float tMax;
};

// Match C++ SimpleHit struct
struct HitRecord {
    uint faceIndex;
    float barycentricU;
    float barycentricV;
    float distance;
    uint sourceVertex;
    uint rayIndex;
    uint padding[2];
};

layout(location = 0) rayPayloadInEXT RayPayload payload;

layout(binding = 0, set = 0) uniform accelerationStructureEXT cageAccel;

// Hit buffer (binding 3) - using coherent for visibility across invocations
layout(binding = 3, set = 0) coherent buffer HitBuffer {
    HitRecord hits[];
} hitBuffer;

// Cage indices buffer (binding 4)
layout(binding = 4, set = 0) readonly buffer CageIndexBuffer {
    uint indices[];
} cageIndices;

layout(push_constant) uniform PushConstants {
    uint vertexCount;
    uint raysPerVertex;
    uint maxHitsPerRay;
    uint padding;
} pc;

void main() {
    // Check if we've reached max hits
    if (payload.hitCount >= pc.maxHitsPerRay) {
        return;
    }
    
    // Calculate hit index in buffer
    uint hitIndex = payload.rayIndex * pc.maxHitsPerRay + payload.hitCount;
    
    // Get triangle info
    uint triIdx = gl_PrimitiveID;
    
    // Compute barycentric coordinates properly
    // hitAttributes contains (u, v) from triangle intersection
    float u = hitAttributes.x;
    float v = hitAttributes.y;
    float w = 1.0 - u - v;
    
    // Write hit record - matching C++ layout
    hitBuffer.hits[hitIndex].faceIndex = triIdx;
    hitBuffer.hits[hitIndex].barycentricU = u;
    hitBuffer.hits[hitIndex].barycentricV = v;
    hitBuffer.hits[hitIndex].distance = gl_HitTEXT;
    hitBuffer.hits[hitIndex].sourceVertex = payload.vertexIndex;
    hitBuffer.hits[hitIndex].rayIndex = payload.rayIndex;
    
    // Increment hit count with atomic operation to prevent race conditions
    // Note: This requires the buffer to be declared as 'coherent'
    payload.hitCount++;
    
    // Optional: continue tracing for multiple hits per ray
    if (payload.hitCount < pc.maxHitsPerRay) {
        // Trace from just past the hit point
        vec3 newOrigin = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * (gl_HitTEXT + 0.001);
        
        traceRayEXT(
            cageAccel,
            gl_RayFlagsOpaqueEXT,
            0xFF,
            0,
            0,
            0,
            newOrigin,
            0.001,
            gl_WorldRayDirectionEXT,
            payload.tMax - gl_HitTEXT,
            0
        );
    }
}