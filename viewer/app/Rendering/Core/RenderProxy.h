#pragma once

#include <Rendering/Core/Device.h>
#include <Rendering/Core/RenderResource.h>
#include <Rendering/Core/Pipeline.h>
#include <Core/Types.h>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

class RenderCommandScheduler;
class RenderResourceManager;

using RenderProxyHandle = std::size_t;
static constexpr auto InvalidHandle = std::numeric_limits<std::size_t>::max();

struct Vertex
{
	Vertex() = default;
	Vertex(const glm::vec3& normal, const glm::vec3& vertexColor)
		: _normal(normal)
		, _vertexColor(vertexColor)
	{ }

	ALIGN_SIZE(16) glm::vec3 _normal { 0.0f };
	ALIGN_SIZE(16) glm::vec3 _vertexColor { 0.0f };
};

struct MeshRenderProxyFlags
{
	MeshRenderProxyFlags()
		: _renderPoints(false)
		, _renderEdges(false)
		, _supportsWireframeRendering(false)
		, _isVisible(true)
	{ }

	uint32_t _renderPoints : 1;
	uint32_t _renderEdges : 1;
	uint32_t _supportsWireframeRendering : 1;
	uint32_t _isVisible : 1;
};

enum class WireframeRenderMode : uint8_t
{
	None = 0,
	Points = 1 << 0,
	Edges = 1 << 1
};

struct MeshProxySolidPipeline
{
	PipelineHandle _handle = InvalidHandle;
	PipelineHandle _vertexColorHandle = InvalidHandle;
	std::array<std::vector<VkDescriptorSet>, VulkanUtils::NumRenderFramesInFlight> _descriptorSets;
};

class RenderProxy
{
public:
	RenderProxy(const RenderResourceRef<Device>& device,
		const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
		MeshProxySolidPipeline solidPipeline,
		const bool supportsWireframeRendering);

	virtual ~RenderProxy() = default;

	/**
	 * Schedules the render commands on the buffer to render the proxy.
	 * @param commandBuffer A command buffer to schedule the commands on.
	 * @param currentFrameIndex The index of the current frame.
	 * @param dynamicBufferOffset The offset into the dynamic objects buffer.
	 */
	virtual void Render(const VkCommandBuffer commandBuffer,
		const uint32_t currentFrameIndex,
		const uint32_t dynamicBufferOffset) = 0;

	/**
	 * Destroys the GPU resources associated with the instance.
	 * @param device A reference to the Vulkan device.
	 */
	virtual void DestroyRenderProxy(const RenderResourceRef<Device>& device) = 0;

	/**
	 * Updates the render proxy if it was dirty and needs to re-allocate resources. This way we can avoid regenerating
	 * all buffers if only a single one has been updated, for example the positions buffer.
	 * @param resourceManager A reference to the render resource manager.
	 */
	virtual void Update(const std::shared_ptr<RenderResourceManager>& resourceManager) { }

	/**
	 * Sets a new model matrix.
	 * @param modelMatrix The new model matrix.
	 */
	void SetModelMatrix(const glm::mat4& modelMatrix)
	{
		_modelMatrix = modelMatrix;
	}

	/**
	 * Gets the model matrix of the proxy.
	 * @return The model matrix of the proxy.
	 */
	[[nodiscard]] glm::mat4 GetModelMatrix() const
	{
		return _modelMatrix;
	}

	/**
	 * Sets the visibility of the object.
	 * @param isVisible The visibility of the proxy.
	 */
	void SetVisible(const bool isVisible)
	{
		_renderFlags._isVisible = isVisible;
	}

	/**
	 * Returns the visibility of the proxy object.
	 * @return The visibility of the proxy object.
	 */
	[[nodiscard]] bool IsVisible() const
	{
		return _renderFlags._isVisible;
	}

protected:
	/// Reference to the device.
	RenderResourceRef<Device> _device;

	/// Weak reference to the render command scheduler so we can schedule updates to certain proxy properties.
	std::weak_ptr<RenderCommandScheduler> _renderCommandScheduler;

	/// Pipeline used for the rendering of the specific mesh.
	MeshProxySolidPipeline _solidPipeline;

	/// The model matrix of the proxy. This is used only for the rendering, the objects have their copy of the matrix for the frame.
	glm::mat4 _modelMatrix { 1.0f };

	/// Settings on how to render the mesh.
	MeshRenderProxyFlags _renderFlags;
};

struct MeshProxyWireframePipeline
{
	PipelineHandle _pointsHandle = InvalidHandle;
	PipelineHandle _edgesHandle = InvalidHandle;
	std::array<std::vector<VkDescriptorSet>, VulkanUtils::NumRenderFramesInFlight> _descriptorSets;
};

class MeshRenderProxy : public RenderProxy
{
public:
	MeshRenderProxy(const RenderResourceRef<Device>& device,
		const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
		MeshProxySolidPipeline solidPipeline,
		MeshProxyWireframePipeline wireframePipeline,
		const bool supportsWireframeRendering,
		const WireframeRenderMode wireframeRenderMode);

	/**
	 * Schedules the render commands on the buffer to render the proxy.
	 * @param commandBuffer A command buffer to schedule the commands on.
	 * @param currentFrameIndex The index of the current frame.
	 * @param dynamicBufferOffset Dynamic byffer offset.
	 */
	void Render(const VkCommandBuffer commandBuffer,
		const uint32_t currentFrameIndex,
		const uint32_t dynamicBufferOffset) override { }

	/**
	 * Destroys the GPU resources associated with the instance.
	 * @param device A reference to the Vulkan device.
	 */
	void DestroyRenderProxy(const RenderResourceRef<Device>& device) override { }

	/**
	 * Updates the render proxy if it was dirty and needs to re-allocate resources. This way we can avoid regenerating
	 * all buffers if only a single one has been updated, for example the positions buffer.
	 * @param resourceManager A reference to the render resource manager.
	 */
	void Update(const std::shared_ptr<RenderResourceManager>& resourceManager) override { }

	/**
	 * Sets a flag whether the mesh should be rendered as wireframe.
	 * @param wireframeRenderMode A flag how the mesh should be rendered in wireframe mode.
	 */
	void SetWireframeRenderMode(const WireframeRenderMode wireframeRenderMode)
	{
		_renderFlags._renderPoints = IsSet(wireframeRenderMode, WireframeRenderMode::Points);
		_renderFlags._renderEdges = IsSet(wireframeRenderMode, WireframeRenderMode::Edges);
	}

protected:
	/// The pipeline used to render the mesh as wireframe with points.
	MeshProxyWireframePipeline _wireframePipeline;
};