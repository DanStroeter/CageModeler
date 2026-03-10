#pragma once
#include <vector>
#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <Vulkan/vulkan.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/PMVC/ScopedCmdBuffer.h>

/*
class SphereWeightCalculator {
public:
    VkImage        _solidAngleImage = VK_NULL_HANDLE;
    VkDeviceMemory _solidAngleMemory = VK_NULL_HANDLE;
    VkImageView    _solidAngleArrayView = VK_NULL_HANDLE;
    VkSampler      _solidAngleSampler = VK_NULL_HANDLE;

    SphereWeightCalculator() = default;

    void Cleanup(RenderResourceRef<Device> device) {
        if (_solidAngleSampler != VK_NULL_HANDLE) {
            vkDestroySampler(device->GetReference(), _solidAngleSampler, nullptr);
            _solidAngleSampler = VK_NULL_HANDLE;
        }
        if (_solidAngleArrayView != VK_NULL_HANDLE) {
            vkDestroyImageView(device->GetReference(), _solidAngleArrayView, nullptr);
            _solidAngleArrayView = VK_NULL_HANDLE;
        }
        if (_solidAngleImage != VK_NULL_HANDLE) {
            vkDestroyImage(device->GetReference(), _solidAngleImage, nullptr);
            _solidAngleImage = VK_NULL_HANDLE;
        }
        if (_solidAngleMemory != VK_NULL_HANDLE) {
            vkFreeMemory(device->GetReference(), _solidAngleMemory, nullptr);
            _solidAngleMemory = VK_NULL_HANDLE;
        }
    }

    void SphereWeightInitialization(uint32_t size,
        RenderResourceRef<Device> device,
        std::shared_ptr<RenderResourceManager> resourceManager,
        VkCommandPool commandPool) {
        // Store parameters for later use
        _faceSize = size;
        _device = device;
        _resourceManager = resourceManager;

        // Calculate all solid angles and store them in a host buffer
        std::vector<float> solidAngles(6 * size * size);

        for (int face = 0; face < 6; ++face) {
            for (int y = 0; y < size; ++y) {
                for (int x = 0; x < size; ++x) {
                    int index = (face * size * size) + (y * size) + x;
                    solidAngles[index] = ComputeSphereWeight(x, y, size, face);
                }
            }
        }

        // Create Vulkan resources for the solid angle texture array
        CreateSolidAngleTexture(size, solidAngles, device, resourceManager, commandPool);
    }

    float ComputeSphereWeight(int px, int py, int faceSize, int face) {
        // Constants
        const double PI = 3.14159265358979323846;

        // Convert pixel coordinates to normalized cube face coordinates [-1, 1]
        // Using pixel centers
        double u = 2.0 * (px + 0.5) / faceSize - 1.0;
        double v = 2.0 * (py + 0.5) / faceSize - 1.0;

        // Get the four corners of the pixel in cube face coordinates
        double u_left = 2.0 * px / faceSize - 1.0;
        double u_right = 2.0 * (px + 1) / faceSize - 1.0;
        double v_bottom = 2.0 * py / faceSize - 1.0;
        double v_top = 2.0 * (py + 1) / faceSize - 1.0;

        // Array of corners in (u,v) coordinates
        struct Point2D { double u, v; };
        Point2D corners_cube[4] = {
            {u_left, v_bottom},
            {u_right, v_bottom},
            {u_right, v_top},
            {u_left, v_top}
        };

        // Map corners to sphere
        //struct Point3D { double x, y, z; };
        Point3D corners_sphere[4];

        for (int i = 0; i < 4; ++i) {
            corners_sphere[i] = FaceToSphere(corners_cube[i].u, corners_cube[i].v, face);
        }

        // Calculate spherical area of the pixel (solid angle)
        double sphere_area = SphericalQuadArea(
            corners_sphere[0], corners_sphere[1],
            corners_sphere[2], corners_sphere[3]
        );

        return static_cast<float>(sphere_area);
    }

private:
    uint32_t _faceSize = 0;
    RenderResourceRef<Device> _device;
    std::shared_ptr<RenderResourceManager> _resourceManager;

    struct Point3D { double x, y, z; };

    Point3D FaceToSphere(double u, double v, int face) const {
        double x, y, z;

        // Map based on cube face
        switch (face) {
        case 0:  // +X
            x = 1.0; y = -v; z = -u;
            break;
        case 1:  // -X
            x = -1.0; y = -v; z = u;
            break;
        case 2:  // +Y
            x = u; y = 1.0; z = v;
            break;
        case 3:  // -Y
            x = u; y = -1.0; z = -v;
            break;
        case 4:  // +Z
            x = u; y = -v; z = 1.0;
            break;
        case 5:  // -Z
            x = -u; y = -v; z = -1.0;
            break;
        default:
            throw std::invalid_argument("Face must be between 0 and 5");
        }

        // Normalize to unit sphere
        double norm = std::sqrt(x * x + y * y + z * z);
        return { x / norm, y / norm, z / norm };
    }

    double Dot(const Point3D& a, const Point3D& b) const {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    Point3D Cross(const Point3D& a, const Point3D& b) const {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    double Norm(const Point3D& p) const {
        return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    double SphericalTriangleArea(const Point3D& p1, const Point3D& p2, const Point3D& p3) const {
        // Using the formula: tan(E/2) = |det(p1,p2,p3)| / (1 + p1*p2 + p1*p3 + p2*p3)
        // where E is the spherical excess (area on unit sphere)

        double dot12 = Dot(p1, p2);
        double dot13 = Dot(p1, p3);
        double dot23 = Dot(p2, p3);

        // Determinant (scalar triple product)
        Point3D cross = Cross(p2, p3);
        double det = Dot(p1, cross);

        double denominator = 1.0 + dot12 + dot13 + dot23;

        if (std::abs(denominator) < 1e-10) {
            return 0.0;
        }

        double tan_half_E = std::abs(det) / denominator;

        // Area = 2 * atan(tan_half_E)
        // For small angles, atan(x) = x, but we use exact formula
        double area = 2.0 * std::atan(tan_half_E);

        // Ensure non-negative
        return std::max(0.0, area);
    }

    double SphericalQuadArea(const Point3D& p1, const Point3D& p2,
        const Point3D& p3, const Point3D& p4) const {
        // Split into two triangles and sum
        double area1 = SphericalTriangleArea(p1, p2, p3);
        double area2 = SphericalTriangleArea(p1, p3, p4);
        return area1 + area2;
    }

    void CreateSolidAngleTexture(uint32_t size,
        const std::vector<float>& solidAngles,
        RenderResourceRef<Device> device,
        std::shared_ptr<RenderResourceManager> resourceManager,
        VkCommandPool commandPool) {
        // Implementation depends on your specific Vulkan setup
        // This is a placeholder - you'll need to adapt this to your rendering system

        VkDevice vkDevice = device->GetReference();
//->GetDeviceHandle();

        // Create staging buffer
        MemoryMappedBuffer stagingBuffer;
        Buffer stagingMemory;

        MemoryMappedBuffer stagingBuffer = _resourceManager->CreateBufferAndCopy(
            std::span<const float>(solidAngles.data(), solidAngles.size()),
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
        );

        Buffer stagingMemory = _resourceManager->AllocateDeviceBuffer(
            std::span<const float>(solidAngles.size()),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
            VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
        );

        CopyBuffer(
            stagingBuffer._deviceBuffer,
            stagingMemory._deviceBuffer,
            solidAngles.size() * sizeof(float),
            commandPool);

        // Create the actual texture image (array of 6 2D textures)
        VkImageCreateInfo imageInfo = {};
        imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.format = VK_FORMAT_R32_SFLOAT;
        imageInfo.extent.width = size;
        imageInfo.extent.height = size;
        imageInfo.extent.depth = 1;
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 6;  // 6 faces
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

        vkCreateImage(vkDevice, &imageInfo, nullptr, &_solidAngleImage);

        VkMemoryRequirements memRequirements;
        vkGetBufferMemoryRequirements(vkDevice, stagingBuffer._deviceBuffer, &memRequirements);

        VkMemoryAllocateInfo allocInfo = {};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memRequirements.size;

        // Allocate memory for the image
        vkGetImageMemoryRequirements(vkDevice, _solidAngleImage, &memRequirements);
        allocInfo.allocationSize = memRequirements.size;
        // Find memory type that is device local
        vkAllocateMemory(vkDevice, &allocInfo, nullptr, &_solidAngleMemory);
        vkBindImageMemory(vkDevice, _solidAngleImage, _solidAngleMemory, 0);

        // Create command buffer for copying
        VkCommandBufferAllocateInfo cmdAllocInfo = {};
        cmdAllocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        cmdAllocInfo.commandPool = commandPool;
        cmdAllocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cmdAllocInfo.commandBufferCount = 1;

        VkCommandBuffer commandBuffer;
        vkAllocateCommandBuffers(vkDevice, &cmdAllocInfo, &commandBuffer);

        // Record commands
        VkCommandBufferBeginInfo beginInfo = {};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        vkBeginCommandBuffer(commandBuffer, &beginInfo);

        // Transition image layout to transfer destination
        VkImageMemoryBarrier barrier = {};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = _solidAngleImage;
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 6;
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

        vkCmdPipelineBarrier(commandBuffer,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            0,
            0, nullptr,
            0, nullptr,
            1, &barrier);

        // Copy buffer to image for each layer
        VkBufferImageCopy region = {};
        region.bufferOffset = 0;
        region.bufferRowLength = 0;
        region.bufferImageHeight = 0;
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.mipLevel = 0;
        region.imageSubresource.baseArrayLayer = 0;
        region.imageSubresource.layerCount = 6;
        region.imageOffset = { 0, 0, 0 };
        region.imageExtent = { size, size, 1 };

        vkCmdCopyBufferToImage(commandBuffer, stagingBuffer, _solidAngleImage,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

        // Transition image layout to shader read only
        barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

        vkCmdPipelineBarrier(commandBuffer,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            0,
            0, nullptr,
            0, nullptr,
            1, &barrier);

        vkEndCommandBuffer(commandBuffer);

        // Submit command buffer
        VkSubmitInfo submitInfo = {};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;

        //VkQueue graphicsQueue = device->GetQueue(TQ_Graphics, 0);
        VkQueue graphicsQueue = device->GetDeviceQueue(
			device->GetQueueFamilies()._graphics.value(),
			0);
        vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
        vkQueueWaitIdle(graphicsQueue);

        // Clean up staging resources
        vkFreeCommandBuffers(vkDevice, commandPool, 1, &commandBuffer);
        vkDestroyBuffer(vkDevice, stagingBuffer, nullptr);
        vkFreeMemory(vkDevice, stagingMemory, nullptr);

        // Create image view
        VkImageViewCreateInfo viewInfo = {};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = _solidAngleImage;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
        viewInfo.format = VK_FORMAT_R32_SFLOAT;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 6;

        vkCreateImageView(vkDevice, &viewInfo, nullptr, &_solidAngleArrayView);

        // Create sampler
        VkSamplerCreateInfo samplerInfo = {};
        samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samplerInfo.magFilter = VK_FILTER_LINEAR;
        samplerInfo.minFilter = VK_FILTER_LINEAR;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.anisotropyEnable = VK_FALSE;
        samplerInfo.maxAnisotropy = 1.0f;
        samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        samplerInfo.unnormalizedCoordinates = VK_FALSE;
        samplerInfo.compareEnable = VK_FALSE;
        samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        samplerInfo.mipLodBias = 0.0f;
        samplerInfo.minLod = 0.0f;
        samplerInfo.maxLod = 0.0f;

        vkCreateSampler(vkDevice, &samplerInfo, nullptr, &_solidAngleSampler);
    }

    void CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size, VkCommandPool commandPool)
    {
        // Allocate a temporary one-time command buffer
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandPool = commandPool;
        allocInfo.commandBufferCount = 1;

        VkCommandBuffer cmd;
        vkAllocateCommandBuffers(_device, &allocInfo, &cmd);

        // Begin recording
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        vkBeginCommandBuffer(cmd, &beginInfo);

        // Copy command
        VkBufferCopy copyRegion{};
        copyRegion.srcOffset = 0;
        copyRegion.dstOffset = 0;
        copyRegion.size = size;
        vkCmdCopyBuffer(cmd, src, dst, 1, &copyRegion);
        vkEndCommandBuffer(cmd);

        // Submit and wait
        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &cmd;

        VkQueue graphicsQueue;
        vkGetDeviceQueue(_device, _device->GetQueueFamilies()._graphics.value(), 0, &graphicsQueue);

        vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
        vkQueueWaitIdle(graphicsQueue);

        // Clean up
        vkFreeCommandBuffers(_device, commandPool, 1, &cmd);
    }
};
*/
class SphereWeightCalculator {
public:
    VkImage        _solidAngleImage = VK_NULL_HANDLE;
    VkDeviceMemory _solidAngleMemory = VK_NULL_HANDLE;
    VkImageView    _solidAngleArrayView = VK_NULL_HANDLE;
    VkSampler      _solidAngleSampler = VK_NULL_HANDLE;

    SphereWeightCalculator() = default;

    void Cleanup(RenderResourceRef<Device> device) {
        if (_solidAngleSampler != VK_NULL_HANDLE) {
            vkDestroySampler(device->GetReference(), _solidAngleSampler, nullptr);
            _solidAngleSampler = VK_NULL_HANDLE;
        }
        if (_solidAngleArrayView != VK_NULL_HANDLE) {
            vkDestroyImageView(device->GetReference(), _solidAngleArrayView, nullptr);
            _solidAngleArrayView = VK_NULL_HANDLE;
        }
        if (_solidAngleImage != VK_NULL_HANDLE) {
            vkDestroyImage(device->GetReference(), _solidAngleImage, nullptr);
            _solidAngleImage = VK_NULL_HANDLE;
        }
        if (_solidAngleMemory != VK_NULL_HANDLE) {
            vkFreeMemory(device->GetReference(), _solidAngleMemory, nullptr);
            _solidAngleMemory = VK_NULL_HANDLE;
        }
    }

    void SphereWeightInitialization(uint32_t size,
        RenderResourceRef<Device> device,
        std::shared_ptr<RenderResourceManager> resourceManager,
        VkCommandPool commandPool) {
        /*
        const uint32_t faceSize = size;
        const uint32_t faceCount = 6;

        // ------------------------------------------------------------
        // 1) CPU: precompute solid-angle weights using efficient analytical formula
        // ------------------------------------------------------------
        std::vector<float> weights(faceCount * faceSize * faceSize);

        for (uint32_t face = 0; face < faceCount; ++face) {
            for (uint32_t y = 0; y < faceSize; ++y) {
                for (uint32_t x = 0; x < faceSize; ++x) {
                    weights[
                        face * faceSize * faceSize +
                            y * faceSize + x
                    ] = TexelCoordSolidAngle(face, x, y, faceSize);
                }
            }
        }
        */
        const uint32_t faceSize = size;
        const uint32_t faceCount = 6;

        // Calculate cube pixel area (constant for all pixels at this resolution)
        float cubePixelArea = (2.0f / size) * (2.0f / size);  // Area on cube face [-1,1] range

        // ------------------------------------------------------------
        // 1) CPU: precompute solid-angle WEIGHT RATIOS
        // ------------------------------------------------------------
        std::vector<float> weights(faceCount * faceSize * faceSize);

        for (uint32_t face = 0; face < faceCount; ++face) {
            for (uint32_t y = 0; y < faceSize; ++y) {
                for (uint32_t x = 0; x < faceSize; ++x) {
                    // Get solid angle in steradians
                    float solidAngle = TexelCoordSolidAngle(face, x, y, faceSize);

                    // Convert to ratio (sphere area / cube area)
                    float ratio = solidAngle / cubePixelArea;

                    weights[
                        face * faceSize * faceSize +
                            y * faceSize + x
                    ] = ratio;  // Store the ratio instead of solid angle
                }
            }
        }

        // ------------------------------------------------------------
        // 2) Create GPU image (R32_SFLOAT, 6 layers)
        // ------------------------------------------------------------
        VkImageCreateInfo img{};
        img.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        img.imageType = VK_IMAGE_TYPE_2D;
        img.format = VK_FORMAT_R32_SFLOAT;
        img.extent = { faceSize, faceSize, 1 };
        img.mipLevels = 1;
        img.arrayLayers = 6;
        img.samples = VK_SAMPLE_COUNT_1_BIT;
        img.tiling = VK_IMAGE_TILING_OPTIMAL;
        img.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        img.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        img.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

        vkCreateImage(device->GetReference(), &img, nullptr, &_solidAngleImage);

        VkMemoryRequirements memReq{};
        vkGetImageMemoryRequirements(device->GetReference(), _solidAngleImage, &memReq);

        VkPhysicalDeviceMemoryProperties memProperties;
        vkGetPhysicalDeviceMemoryProperties(device->GetPhysicalDeviceHandle(), &memProperties);

        std::optional<uint32_t> memtype;

        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
            if (memReq.memoryTypeBits & (1 << i)) {
                if (memProperties.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) {
                    memtype = i;
                    break;
                }
            }
        }

        // Fallback: accept any compatible memory
        if (!memtype.has_value()) {
            for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
                if (memReq.memoryTypeBits & (1 << i)) {
                    memtype = i;
                    break;
                }
            }
        }

        if (!memtype.has_value()) {
            throw std::runtime_error("Failed to find ANY compatible memory type for solid angle image!");
        }

        VkMemoryAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        alloc.allocationSize = memReq.size;
        alloc.memoryTypeIndex = memtype.value();

        vkAllocateMemory(device->GetReference(), &alloc, nullptr, &_solidAngleMemory);
        vkBindImageMemory(device->GetReference(), _solidAngleImage, _solidAngleMemory, 0);

        // ------------------------------------------------------------
        // 3) Upload via staging buffer using resourceManager
        // ------------------------------------------------------------
        auto staging = resourceManager->CreateBufferAndCopy(
            std::span<const float>(weights.data(), weights.size()),
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
        );

        // Use ScopedCmdBuffer for command buffer management
        ScopedCmdBuffer scoped(device, commandPool);
        VkCommandBuffer cmd = scoped.Get();

        // Transition image to TRANSFER_DST_OPTIMAL
        VkImageMemoryBarrier barrier1{};
        barrier1.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier1.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        barrier1.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier1.srcAccessMask = 0;
        barrier1.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier1.image = _solidAngleImage;
        barrier1.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier1.subresourceRange.baseMipLevel = 0;
        barrier1.subresourceRange.levelCount = 1;
        barrier1.subresourceRange.baseArrayLayer = 0;
        barrier1.subresourceRange.layerCount = 6;

        vkCmdPipelineBarrier(
            cmd,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            0,
            0, nullptr,
            0, nullptr,
            1, &barrier1
        );

        // Copy buffer to image (all 6 layers in one operation)
        VkBufferImageCopy copy{};
        copy.bufferOffset = 0;
        copy.bufferRowLength = 0;
        copy.bufferImageHeight = 0;
        copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        copy.imageSubresource.mipLevel = 0;
        copy.imageSubresource.baseArrayLayer = 0;
        copy.imageSubresource.layerCount = 6;
        copy.imageOffset = { 0, 0, 0 };
        copy.imageExtent = { faceSize, faceSize, 1 };

        vkCmdCopyBufferToImage(
            cmd,
            staging._deviceBuffer,
            _solidAngleImage,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            1,
            &copy
        );

        // Transition image to SHADER_READ_ONLY_OPTIMAL for compute shader access
        VkImageMemoryBarrier barrier2{};
        barrier2.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier2.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier2.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier2.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier2.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        barrier2.image = _solidAngleImage;
        barrier2.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier2.subresourceRange.baseMipLevel = 0;
        barrier2.subresourceRange.levelCount = 1;
        barrier2.subresourceRange.baseArrayLayer = 0;
        barrier2.subresourceRange.layerCount = 6;

        vkCmdPipelineBarrier(
            cmd,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            0,
            0, nullptr,
            0, nullptr,
            1, &barrier2
        );

        // Submit and wait for completion
        VkQueue transferQueue;
        vkGetDeviceQueue(
            device->GetReference(),
            device->GetQueueFamilies()._graphics.value(),
            0,
            &transferQueue
        );
        scoped.SubmitAndWait(transferQueue);

        // ------------------------------------------------------------
        // 4) Create 2D-array image view (for compute)
        // ------------------------------------------------------------
        VkImageViewCreateInfo view{};
        view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        view.image = _solidAngleImage;
        view.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
        view.format = VK_FORMAT_R32_SFLOAT;
        view.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        view.subresourceRange.baseMipLevel = 0;
        view.subresourceRange.levelCount = 1;
        view.subresourceRange.baseArrayLayer = 0;
        view.subresourceRange.layerCount = 6;

        vkCreateImageView(device->GetReference(), &view, nullptr, &_solidAngleArrayView);

        // ------------------------------------------------------------
        // 5) Create sampler (NEAREST - we want exact pixel values, no interpolation)
        // ------------------------------------------------------------
        VkSamplerCreateInfo samp{};
        samp.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samp.magFilter = VK_FILTER_NEAREST;
        samp.minFilter = VK_FILTER_NEAREST;
        samp.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        samp.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samp.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samp.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samp.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        samp.unnormalizedCoordinates = VK_FALSE;
        samp.compareEnable = VK_FALSE;
        samp.compareOp = VK_COMPARE_OP_ALWAYS;
        samp.mipLodBias = 0.0f;
        samp.minLod = 0.0f;
        samp.maxLod = 0.0f;
        samp.anisotropyEnable = VK_FALSE;
        samp.maxAnisotropy = 1.0f;

        vkCreateSampler(device->GetReference(), &samp, nullptr, &_solidAngleSampler);

        // Optional: Verify total solid angle (should be 4?)
        double sum = 0.0;
        for (float w : weights) sum += w;
        // LOG_DEBUG("Total solid angle = " + std::to_string(sum));
        // Expected: 4? ? 12.56637
    }

    // The efficient analytical formula for solid angle of a cube map texel
    static float AreaElement(float x, float y) {
        return atan2(x * y, sqrt(x * x + y * y + 1.0f));
    }

    float TexelCoordSolidAngle(int faceIdx, int texelX, int texelY, int size) const {
        // Scale up to [-1, 1] range (inclusive), offset by 0.5 to point to texel center
        float U = (2.0f * ((float)texelX + 0.5f) / (float)size) - 1.0f;
        float V = (2.0f * ((float)texelY + 0.5f) / (float)size) - 1.0f;

        float InvResolution = 1.0f / size;

        // U and V are the -1..1 texture coordinate on the current face
        // Get projected area for this texel
        float x0 = U - InvResolution;
        float y0 = V - InvResolution;
        float x1 = U + InvResolution;
        float y1 = V + InvResolution;

        float solidAngle = AreaElement(x0, y0) - AreaElement(x0, y1)
            - AreaElement(x1, y0) + AreaElement(x1, y1);

        return solidAngle;
    }

    // Convenience wrapper that matches your original ComputeSphereWeight signature
    float ComputeSphereWeight(int px, int py, int faceSize, int face) {
        return TexelCoordSolidAngle(face, px, py, faceSize);
    }

private:
    // No need for complex Point3D structures anymore!
};