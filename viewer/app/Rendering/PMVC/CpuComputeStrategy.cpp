#include <Rendering/PMVC/CpuComputeStrategy.h>
#include <Rendering/PMVC/ScopedCmdBuffer.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <cassert>
#include <cstddef>
#include <thread>


uint32_t CpuComputeStrategy::RequiredRenderTargetCount() const
{
    return std::min<uint32_t>(static_cast<uint32_t>(_deformableMesh._vertices.rows()), _maxTargetCount);
}

void CpuComputeStrategy::Initialize()
{
    _targetCount = static_cast<int>(RequiredRenderTargetCount());
    _slotToDeformableIndex.assign(_targetCount, UINT32_MAX);

    const uint32_t hardwareThreads = std::max(1u, std::thread::hardware_concurrency());
    _cpuWorkerCount = std::max(1u, std::min<uint32_t>(hardwareThreads, static_cast<uint32_t>(_targetCount)));
    _threadPool = std::make_unique<ThreadPool>(_cpuWorkerCount);

    _lambdaResults.resize(_deformableMesh._vertices.rows(), _cageMesh._vertices.rows());
    _lambdaResults.setZero();
    _wsumResults.assign(_deformableMesh._vertices.rows(), 0.0f);

    _depthFormat = _device->FindDepthFormat();
    _depthBytesPerTexel = (_depthFormat == VK_FORMAT_D24_UNORM_S8_UINT) ? sizeof(uint32_t) : sizeof(float);

    _solidAngles.resize(static_cast<size_t>(_faceSize) * _faceSize * 6);
    for (uint32_t face = 0; face < 6; ++face)
    {
        for (uint32_t y = 0; y < _faceSize; ++y)
        {
            for (uint32_t x = 0; x < _faceSize; ++x)
            {
                const size_t texelIdx = static_cast<size_t>(face) * _faceSize * _faceSize + static_cast<size_t>(y) * _faceSize + x;
                _solidAngles[texelIdx] = ComputeSolidAngle(x, y);
            }
        }
    }

    VkCommandPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = _transferQueueFamily;
    VK_CHECK(vkCreateCommandPool(_device, &poolInfo, nullptr, &_computeCommandPool));

    _computeCommandBuffers.resize(_targetCount);
    VkCommandBufferAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    allocInfo.commandPool = _computeCommandPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = static_cast<uint32_t>(_computeCommandBuffers.size());
    VK_CHECK(vkAllocateCommandBuffers(_device, &allocInfo, _computeCommandBuffers.data()));

    AllocateResources();
}

void CpuComputeStrategy::Cleanup()
{
    if (_computeCommandPool != VK_NULL_HANDLE)
    {
        if (!_computeCommandBuffers.empty())
        {
            vkFreeCommandBuffers(_device, _computeCommandPool, static_cast<uint32_t>(_computeCommandBuffers.size()), _computeCommandBuffers.data());
            _computeCommandBuffers.clear();
        }

        vkDestroyCommandPool(_device, _computeCommandPool, nullptr);
        _computeCommandPool = VK_NULL_HANDLE;
    }

    for (auto& slot : _slots)
    {
        if (slot.colorBuffer != VK_NULL_HANDLE)
        {
            vkDestroyBuffer(_device, slot.colorBuffer, nullptr);
            slot.colorBuffer = VK_NULL_HANDLE;
        }
        if (slot.colorMemory != VK_NULL_HANDLE)
        {
            vkFreeMemory(_device, slot.colorMemory, nullptr);
            slot.colorMemory = VK_NULL_HANDLE;
        }

        if (slot.depthBuffer != VK_NULL_HANDLE)
        {
            vkDestroyBuffer(_device, slot.depthBuffer, nullptr);
            slot.depthBuffer = VK_NULL_HANDLE;
        }
        if (slot.depthMemory != VK_NULL_HANDLE)
        {
            vkFreeMemory(_device, slot.depthMemory, nullptr);
            slot.depthMemory = VK_NULL_HANDLE;
        }
    }
}

Eigen::MatrixXd CpuComputeStrategy::Readback()
{
    for (Eigen::Index row = 0; row < _lambdaResults.rows(); ++row)
    {
        if (_wsumResults[static_cast<size_t>(row)] > std::numeric_limits<float>::epsilon())
        {
            _lambdaResults.row(row) /= _wsumResults[static_cast<size_t>(row)];
        }
    }
    return _lambdaResults;
}

void CpuComputeStrategy::AllocateResources()
{
    _slots.resize(_targetCount);

    const VkDeviceSize colorSize = VkDeviceSize(_faceSize) * _faceSize * 6 * 4 * sizeof(float);
    const VkDeviceSize depthSize = VkDeviceSize(_faceSize) * _faceSize * 6 * _depthBytesPerTexel;

    for (int i = 0; i < _targetCount; ++i)
    {
        auto color = _resourceManager->CreateBufferAndMapMemory(
            std::span<std::byte>((std::byte*)nullptr, colorSize),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

        _slots[i].colorBuffer = color._deviceBuffer;
        _slots[i].colorMemory = color._deviceMemory;
        _slots[i].colorMapped = color._mappedData;

        if (!_offset) {
            auto depth = _resourceManager->CreateBufferAndMapMemory(
                std::span<std::byte>((std::byte*)nullptr, depthSize),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

            _slots[i].depthBuffer = depth._deviceBuffer;
            _slots[i].depthMemory = depth._deviceMemory;
            _slots[i].depthMapped = depth._mappedData;
        }
    }

    const int triCount = _cageMesh._faces.rows();
    _vertexList.resize(triCount * 3);
    for (int t = 0; t < triCount; ++t)
    {
        _vertexList[t * 3 + 0] = static_cast<uint32_t>(_cageMesh._faces(t, 0));
        _vertexList[t * 3 + 1] = static_cast<uint32_t>(_cageMesh._faces(t, 1));
        _vertexList[t * 3 + 2] = static_cast<uint32_t>(_cageMesh._faces(t, 2));
    }

    /*
    auto vertexListStaging = _resourceManager->CreateBufferAndCopy(
        std::span<const uint32_t>(_vertexList.data(), _vertexList.size()),
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    _vertexListBuffer = _resourceManager->AllocateDeviceBuffer(
        _vertexList.size() * sizeof(uint32_t),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    CopyBuffer(vertexListStaging._deviceBuffer, _vertexListBuffer._deviceBuffer, _vertexList.size() * sizeof(uint32_t));
    */
}

void CpuComputeStrategy::CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size)
{
    ScopedCmdBuffer scoped(_device, _computeCommandPool);
    VkCommandBuffer cmd = scoped.Get();

    VkBufferCopy copyRegion{};
    copyRegion.size = size;
    vkCmdCopyBuffer(cmd, src, dst, 1, &copyRegion);

    VkQueue queue;
    vkGetDeviceQueue(_device, _transferQueueFamily, 0, &queue);

    scoped.SubmitAndWait(queue);
}

void CpuComputeStrategy::RecordReadback(
    uint32_t slot,
    const CubemapRenderTarget& target,
    uint32_t deformableIndex)
{
    assert(slot < _computeCommandBuffers.size());
    assert(slot < _slots.size());
    assert(slot < _slotToDeformableIndex.size());

    _slotToDeformableIndex[slot] = deformableIndex;

    VkCommandBuffer cmd = _computeCommandBuffers[slot];
    VK_CHECK(vkResetCommandBuffer(cmd, 0));

    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo));

    const VkDeviceSize colorFaceSize = VkDeviceSize(_faceSize) * _faceSize * 4 * sizeof(float);
    const VkDeviceSize depthFaceSize = VkDeviceSize(_faceSize) * _faceSize * _depthBytesPerTexel;

    std::array<VkBufferImageCopy, 6> colorRegions{};
    std::array<VkBufferImageCopy, 6> depthRegions{};

    for (uint32_t face = 0; face < 6; ++face)
    {
        colorRegions[face].bufferOffset = colorFaceSize * face;
        colorRegions[face].imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        colorRegions[face].imageSubresource.baseArrayLayer = face;
        colorRegions[face].imageSubresource.layerCount = 1;
        colorRegions[face].imageExtent = { _faceSize, _faceSize, 1 };

        if (!_offset) {
            depthRegions[face].bufferOffset = depthFaceSize * face;
            depthRegions[face].imageSubresource.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
            depthRegions[face].imageSubresource.baseArrayLayer = face;
            depthRegions[face].imageSubresource.layerCount = 1;
            depthRegions[face].imageExtent = { _faceSize, _faceSize, 1 };
        }
    }

    vkCmdCopyImageToBuffer(
        cmd,
        target.cubemapImage,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        _slots[slot].colorBuffer,
        static_cast<uint32_t>(colorRegions.size()),
        colorRegions.data());

    if (!_offset) {
        vkCmdCopyImageToBuffer(
            cmd,
            target.depthImage,
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            _slots[slot].depthBuffer,
            static_cast<uint32_t>(depthRegions.size()),
            depthRegions.data());
    }

    VK_CHECK(vkEndCommandBuffer(cmd));
}

void CpuComputeStrategy::SubmitAllReadbacks(
    VkSemaphore waitSemaphore,
    uint64_t waitValue,
    VkSemaphore signalSemaphore,
    uint64_t signalValue)
{
    std::vector<VkCommandBufferSubmitInfo> cmdInfos;
    cmdInfos.reserve(_computeCommandBuffers.size());
    for (size_t i = 0; i < _computeCommandBuffers.size(); ++i)
    {
        if (_slotToDeformableIndex[i] == UINT32_MAX)
        {
            continue;
        }

        cmdInfos.push_back({
            .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO,
            .commandBuffer = _computeCommandBuffers[i]
            });
    }

    if (cmdInfos.empty())
    {
        return;
    }

    VkSemaphoreSubmitInfo waitInfo{
        .sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
        .semaphore = waitSemaphore,
        .value = waitValue,
        .stageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT
    };

    VkSemaphoreSubmitInfo signalInfo{
        .sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
        .semaphore = signalSemaphore,
        .value = signalValue,
        .stageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT
    };

    VkSubmitInfo2 submit2{
        .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2,
        .waitSemaphoreInfoCount = 1,
        .pWaitSemaphoreInfos = &waitInfo,
        .commandBufferInfoCount = static_cast<uint32_t>(cmdInfos.size()),
        .pCommandBufferInfos = cmdInfos.data(),
        .signalSemaphoreInfoCount = 1,
        .pSignalSemaphoreInfos = &signalInfo
    };

    VkQueue queue;
    vkGetDeviceQueue(_device, _transferQueueFamily, 0, &queue);
    VK_CHECK(vkQueueSubmit2(queue, 1, &submit2, VK_NULL_HANDLE));
}

void CpuComputeStrategy::ConsumeAllSlots()
{
    std::vector<std::future<void>> tasks;
    tasks.reserve(_slots.size());

    for (size_t slot = 0; slot < _slots.size(); ++slot)
    {
        const uint32_t deformableIndex = _slotToDeformableIndex[slot];
        if (deformableIndex == UINT32_MAX)
        {
            continue;
        }

        tasks.emplace_back(_threadPool->Submit([this, deformableIndex, slot]()
        {
            ComputeOnCpu(deformableIndex, _slots[slot]);
        }));
    }

    for (auto& task : tasks)
    {
        task.get();
    }

    for (size_t slot = 0; slot < _slots.size(); ++slot)
    {
        if (_slotToDeformableIndex[slot] != UINT32_MAX)
        {
            _slotToDeformableIndex[slot] = UINT32_MAX;
        }
    }
}

static float AreaElement(float x, float y) {
    return atan2(x * y, sqrt(x * x + y * y + 1.0f));
}

float CpuComputeStrategy::ComputeSolidAngle(uint32_t texelX, uint32_t texelY) const
{
    const float size = static_cast<float>(_faceSize);
    const float u = (2.0f * (static_cast<float>(texelX) + 0.5f) / size) - 1.0f;
    const float v = (2.0f * (static_cast<float>(texelY) + 0.5f) / size) - 1.0f;
    const float invResolution = 1.0f / size;

    const float x0 = u - invResolution;
    const float y0 = v - invResolution;
    const float x1 = u + invResolution;
    const float y1 = v + invResolution;

    return AreaElement(x0, y0) - AreaElement(x0, y1) - AreaElement(x1, y0) + AreaElement(x1, y1);
}

float CpuComputeStrategy::DecodeDepthSample(const uint8_t* texel) const
{
    switch (_depthFormat)
    {
    case VK_FORMAT_D32_SFLOAT:
    case VK_FORMAT_D32_SFLOAT_S8_UINT:
        return *reinterpret_cast<const float*>(texel);
    case VK_FORMAT_D24_UNORM_S8_UINT:
    {
        const uint32_t packed = *reinterpret_cast<const uint32_t*>(texel);
        const uint32_t depth24 = packed & 0x00FFFFFFu;
        return static_cast<float>(depth24) / 16777215.0f;
    }
    default:
        return 1.0f;
    }
}

void CpuComputeStrategy::ComputeOnCpu(uint32_t deformableIndex, const SlotReadback& slot)
{
    const float* colors = reinterpret_cast<const float*>(slot.colorMapped);
    const uint8_t* depthBytes = reinterpret_cast<const uint8_t*>(slot.depthMapped);

    std::vector<float> lambda(static_cast<size_t>(_cageMesh._vertices.rows()), 0.0f);
    float wsum = 0.0f;

    const uint32_t numTriangles = static_cast<uint32_t>(_cageMesh._faces.rows());
    const size_t texelCountPerFace = static_cast<size_t>(_faceSize) * _faceSize;

    for (uint32_t face = 0; face < 6; ++face)
    {
        for (uint32_t y = 0; y < _faceSize; ++y)
        {
            for (uint32_t x = 0; x < _faceSize; ++x)
            {
                const size_t texelIdx = static_cast<size_t>(face) * texelCountPerFace + static_cast<size_t>(y) * _faceSize + x;
                const size_t colorBase = texelIdx * 4;

                const float b0 = colors[colorBase + 0];
                const float b1 = colors[colorBase + 1];
                const float b2 = colors[colorBase + 2];

                const uint32_t tri = static_cast<uint32_t>(colors[colorBase + 3] * float(numTriangles) + 0.5f);
                if (tri >= numTriangles)
                {
                    continue;
                }
                float w = _solidAngles[texelIdx];
                if (!_offset) {
                    float depth = DecodeDepthSample(depthBytes + texelIdx * _depthBytesPerTexel);
                    if (depth >= 0.999999f) //kdepthepsilon
                    {
                        continue;
                    }

                    if (UseInteriorDistance())
                    {
                        // Mirror of PMVCComputeInteriorDist.comp: reconstruct the eye-space
                        // depth of the rasterized hit, lengthen the ray by the barycentric
                        // interpolation of the cage-vertex interior detours and re-encode
                        // through the same [0, 1] perspective depth mapping. Zero detour
                        // (locally convex cage) reproduces the Euclidean weighting exactly.
                        const uint32_t v0 = _vertexList[tri * 3 + 0];
                        const uint32_t v1 = _vertexList[tri * 3 + 1];
                        const uint32_t v2 = _vertexList[tri * 3 + 2];

                        const Eigen::MatrixXf& table = *_interiorDistance.detours;
                        const float detour =
                            b0 * table(v0, deformableIndex) +
                            b1 * table(v1, deformableIndex) +
                            b2 * table(v2, deformableIndex);

                        const float nearPlane = _interiorDistance.nearPlane;
                        const float farPlane = _interiorDistance.farPlane;
                        const float zEye = (farPlane * nearPlane) / (farPlane - depth * (farPlane - nearPlane));

                        const float size = static_cast<float>(_faceSize);
                        const float u = (2.0f * (static_cast<float>(x) + 0.5f) / size) - 1.0f;
                        const float v = (2.0f * (static_cast<float>(y) + 0.5f) / size) - 1.0f;
                        const float cosTheta = 1.0f / std::sqrt(1.0f + u * u + v * v);
                        const float zEyeInterior = zEye + std::max(detour, 0.f) * cosTheta;

                        depth = std::clamp(
                            (farPlane * (zEyeInterior - nearPlane)) / (zEyeInterior * (farPlane - nearPlane)),
                            0.0f, 1.0f);
                    }

                    w *= (1.0f - depth);
                }
                if (w <= 0.0f)
                {
                    continue;
                }

                const uint32_t i0 = _vertexList[tri * 3 + 0];
                const uint32_t i1 = _vertexList[tri * 3 + 1];
                const uint32_t i2 = _vertexList[tri * 3 + 2];

                lambda[i0] += b0 * w;
                lambda[i1] += b1 * w;
                lambda[i2] += b2 * w;
                wsum += w;
            }
        }
    }

    for (size_t c = 0; c < lambda.size(); ++c)
    {
        _lambdaResults(deformableIndex, static_cast<Eigen::Index>(c)) = lambda[c];
    }
    _wsumResults[deformableIndex] = wsum;
}
