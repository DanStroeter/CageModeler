#pragma once

#include <Rendering/Core/RenderResource.h>
#include <Rendering/Core/Device.h>

class Fence : public RenderResource<Fence>
{
public:
	Fence(const RenderResourceRef<Device>& device, const VkFenceCreateFlags fenceFlags)
		: _device(device)
	{
		CHECK_VK_HANDLE(_device);

		VkFenceCreateInfo fenceInfo { };
		fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		fenceInfo.flags = fenceFlags;

		VK_CHECK(vkCreateFence(_device, &fenceInfo, nullptr, &_handle));
	}

	[[nodiscard]] VkFence GetReference() const
	{
		return _handle;
	}

	void AddRef()
	{
		// Nothing to do here.
	}

	void Release()
	{
		vkDestroyFence(_device, _handle, nullptr);
	}

private:
	VkFence _handle = VK_NULL_HANDLE;

	RenderResourceRef<Device> _device;
};

class Semaphore : public RenderResource<Semaphore>
{
public:
	explicit Semaphore(const RenderResourceRef<Device>& device)
		: _device(device)
	{
		CHECK_VK_HANDLE(_device);

		VkSemaphoreCreateInfo semaphoreInfo { };
		semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

		VK_CHECK(vkCreateSemaphore(_device, &semaphoreInfo, nullptr, &_handle));
	}

	[[nodiscard]] VkSemaphore GetReference() const
	{
		return _handle;
	}

	void AddRef()
	{
		// Nothing to do here.
	}

	void Release()
	{
		vkDestroySemaphore(_device, _handle, nullptr);
	}

private:
	VkSemaphore _handle = VK_NULL_HANDLE;

	RenderResourceRef<Device> _device;
};
