#pragma once

#include <volk.h>

enum class ShaderModuleType: uint8_t
{
	Vertex,
	Geometry,
	Fragment
};

struct PipelineObject
{
	PipelineObject() = default;
	PipelineObject(const VkPipeline pipelineHandle, const VkPipelineLayout pipelineLayout)
		: _handle(pipelineHandle)
		, _pipelineLayout(pipelineLayout)
	{ }

	VkPipeline _handle = VK_NULL_HANDLE;
	VkPipelineLayout _pipelineLayout = VK_NULL_HANDLE;
};

class PipelineHandle
{
public:
	PipelineHandle() = default;
	PipelineHandle(const std::size_t index)
		: _index(index)
	{ }

	std::size_t GetIndex() const
	{
		return _index;
	}

	bool IsValid() const
	{
		return (_index == InvalidIndex);
	}

private:
	static constexpr std::size_t InvalidIndex = -1;

	std::size_t _index = InvalidIndex;
};
