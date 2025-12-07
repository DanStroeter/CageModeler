#version 450

#define MAX_CAGE_VERTICES 4096

// barycentric + tri-index texture
// RGBA32F : rgb = barycentrics, a = triangle index
layout(binding = 0) uniform sampler2DArray uBaryTex;

// Triangle → vertex index list
layout(std430, binding = 1) buffer VertexList {
    uint vertexList[];
};

// Output λ values
layout(std430, binding = 2) buffer LambdaOut {
    float lambda[];
};

// Output wsum per cubemap
layout(std430, binding = 3) buffer WSumOut {
    float wsum[];
};

// Uniforms
layout(location = 0) uniform int uNumCubemaps;
layout(location = 1) uniform int uNumCageVertices;
layout(location = 2) uniform ivec2 uFaceSize;          // width, height
layout(location = 3) uniform int uFacesPerCubemap;     // always 6

// One invocation per cubemap
layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

void main()
{
    uint cubemapIdx = gl_GlobalInvocationID.x;
    if (cubemapIdx >= uint(uNumCubemaps)) return;

    // local accumulation buffer
    float accum[MAX_CAGE_VERTICES];
    for (int i = 0; i < uNumCageVertices; ++i)
        accum[i] = 0.0;

    float wsum_local = 0.0;

    int width  = uFaceSize.x;
    int height = uFaceSize.y;

    for (int face = 0; face < uFacesPerCubemap; ++face) {
        int layer = int(cubemapIdx) * uFacesPerCubemap + face;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {

                // fetch barycentrics + tri index in alpha
                vec4 tex = texelFetch(uBaryTex, ivec3(x, y, layer), 0);
                float b0 = tex.r;
                float b1 = tex.g;
                float b2 = tex.b;

                uint tri = uint(tex.a);

                // skip invalid
                if (tri == 0xFFFFFFFFu) continue;

                // fetch the triangle's 3 cage vertex indices
                uint i0 = vertexList[tri * 3u + 0u];
                uint i1 = vertexList[tri * 3u + 1u];
                uint i2 = vertexList[tri * 3u + 2u];

                // weight = 1.0
                float w = 1.0;

                if (i0 < uint(uNumCageVertices)) accum[i0] += b0 * w;
                if (i1 < uint(uNumCageVertices)) accum[i1] += b1 * w;
                if (i2 < uint(uNumCageVertices)) accum[i2] += b2 * w;

                wsum_local += w;
            }
        }
    }

    // write raw sums
    uint base = cubemapIdx * uint(uNumCageVertices);
    for (int i = 0; i < uNumCageVertices; ++i)
        lambda[base + uint(i)] = accum[i];

    wsum[cubemapIdx] = wsum_local;

    // normalize
    if (wsum_local > 0.0) {
        for (int i = 0; i < uNumCageVertices; ++i)
            lambda[base + uint(i)] /= wsum_local;
    }
}
