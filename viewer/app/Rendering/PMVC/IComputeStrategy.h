#pragma once
#define VK_NO_PROTOTYPES
#include <cstdint>
#include <string>
#include <vulkan/vulkan.h>
#include <glm/ext/vector_int2.hpp>

#include <Eigen/Core>

struct CubemapRenderTarget;

struct ComputePushConstants {
    glm::ivec2 uFaceSize;
    int uNumTriangles;
    // Only read by the interior-distance shader variant; the other compute shaders
    // declare a smaller push constant block and ignore the extra fields.
    int uNumCageVertices;
    int uMeshVertexIdx;
    float uNearPlane;
    float uFarPlane;
};

/**
 * Configuration for the interior-distance PMVC variant: when a distance table is set the
 * strategies swap the depth-based (Euclidean) weighting for the barycentric interpolation
 * of the precomputed heat-method interior distances. The table has one row per cage
 * vertex and one column per deformable mesh vertex.
 */
struct InteriorDistanceSettings {
    const Eigen::MatrixXf* distances = nullptr;
    float nearPlane = 1e-4f;
    float farPlane = 1.0f;

    [[nodiscard]] bool IsEnabled() const
    {
        return distances != nullptr && distances->size() > 0;
    }
};

class ICubemapComputeStrategy
{
public:
    virtual ~ICubemapComputeStrategy() = default;

    virtual uint32_t RequiredRenderTargetCount() const = 0;

    virtual void Initialize() { };

    virtual void Cleanup() { };

    /// Must be called before Initialize to take effect.
    virtual void SetInteriorDistance(const InteriorDistanceSettings&) { };

    /*virtual void DispatchAfterRender(
        uint32_t deformableIndex,
        uint32_t slot,
        VkSemaphore timeline,
        const CubemapRenderTarget& target) = 0;

    virtual void Readback() {};

    virtual void ConsumeSlot(
        uint32_t deformableIndex,
        uint32_t slot,
        VkSemaphore timeline) = 0;

    virtual void SubmitReadbackCopy(
        uint32_t slot,
        VkSemaphore timeline) {
    };*/
};
