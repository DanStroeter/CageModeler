#version 460
#extension GL_EXT_ray_tracing : require

struct RayPayload {
    uint vertexIndex;
    uint rayIndex;
    uint hitCount;
    float tMax;
};

layout(location = 0) rayPayloadInEXT RayPayload payload;

void main() {
    // No hit - just return
    // The hitCount remains unchanged
    // This miss shader is called when a ray doesn't hit anything
}