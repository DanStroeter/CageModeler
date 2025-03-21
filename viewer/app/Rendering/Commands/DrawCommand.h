#pragma once

#include <Rendering/Commands/RenderCommand.h>
#include <Rendering/Utils/VulkanUtils.h>

class BindVertexBuffersCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	using RenderCommand::RenderCommand;

	/**
	 * Binds a pair of vertex and index buffers to the current pipeline.
	 * @param commandBuffer The current command buffer to record.
	 * @param vertexBuffers The positions and vertex buffers as expected in the shader layout.
	 * @param indexBuffer The index buffer of the mesh.
	 */
	void Execute(const VkCommandBuffer commandBuffer,
		const std::span<VkBuffer> vertexBuffers,
		const VkBuffer indexBuffer) const
	{
		CHECK_VK_HANDLE(_commandPool);

		// Initialize a new vector of buffer offsets in the vertex buffers and initialize them to 0.
		// Works only for static meshes with a single buffer for each.
		const std::vector<VkDeviceSize> bufferOffsets(vertexBuffers.size(), 0);

		vkCmdBindVertexBuffers(commandBuffer, 0, bufferOffsets.size(), vertexBuffers.data(), bufferOffsets.data());
		vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT32);
	}
};

class BindGraphicsPipelineCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	using RenderCommand::RenderCommand;

	/**
	 * Record binding of a graphics pipeline object on the command buffer.
	 * @param commandBuffer The current command buffer to record.
	 * @param pipelineHandle The handle to the pipeline to use.
	 */
	void Execute(const VkCommandBuffer commandBuffer,
		const PipelineHandle pipelineHandle) const
	{
		CHECK_VK_HANDLE(_commandPool);

		const auto pipelineObject = GetPipelineObject(pipelineHandle);
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineObject._handle);
	}
};

class BindDescriptorSetCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	using RenderCommand::RenderCommand;

	/**
	 * Record binding of descriptor sets on the command buffer.
	 * @param commandBuffer The current command buffer to record.
	 * @param pipelineHandle The handle to the pipeline to use.
	 * @param descriptorSets The descriptor sets to bind.
	 * @param dynamicOffsets The dynamic offsets into the buffer.
	 */
	void Execute(const VkCommandBuffer commandBuffer,
		const PipelineHandle pipelineHandle,
		const std::span<VkDescriptorSet> descriptorSets,
		const std::span<std::uint32_t> dynamicOffsets) const
	{
		CHECK_VK_HANDLE(_commandPool);

		const auto pipelineObject = GetPipelineObject(pipelineHandle);
		vkCmdBindDescriptorSets(commandBuffer,
			VK_PIPELINE_BIND_POINT_GRAPHICS,
			pipelineObject._pipelineLayout,
			0,
			descriptorSets.size(),
			descriptorSets.data(),
			dynamicOffsets.size(),
			dynamicOffsets.data());
	}
};

class DrawIndexedCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	using RenderCommand::RenderCommand;

	/**
	 * Draw a mesh by index. Requires that you have bound vertex and index buffers before that.
	 * @param commandBuffer The current command buffer to record.
	 * @param indexCount The number of indices to draw.
	 * @param firstIndex The first index to be drawn.
	 */
	void Execute(const VkCommandBuffer commandBuffer,
		const uint32_t indexCount,
		const uint32_t firstIndex) const
	{
		CHECK_VK_HANDLE(_commandPool);

		vkCmdDrawIndexed(commandBuffer, indexCount, 1, firstIndex, 0, 0);
	}
};

class DrawCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	using RenderCommand::RenderCommand;

	/**
	 * Draw a mesh by vertex count which implies the vertex data is either already bound or contained in the shader and there is no index buffer.
	 * @param commandBuffer The current command buffer to record.
	 * @param vertexCount The number of vertices to draw.
	 * @param firstVertex The index of the first vertex to start from.
	 */
	void Execute(const VkCommandBuffer commandBuffer,
		const uint32_t vertexCount,
		const uint32_t firstVertex) const
	{
		CHECK_VK_HANDLE(_commandPool);

		vkCmdDraw(commandBuffer, vertexCount, 1, firstVertex, 0);
	}
};
