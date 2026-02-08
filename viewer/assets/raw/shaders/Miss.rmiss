#version 460
#extension GL_EXT_ray_tracing : require

layout(location = 0) rayPayloadInEXT struct {
    uint vertexIndex;
    uint rayIndex;
    uint hitCount;
    float tMax;
} payload;

void main() {
    // Ray missed - nothing to do for MVC
}