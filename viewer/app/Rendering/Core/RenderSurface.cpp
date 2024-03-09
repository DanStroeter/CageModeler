#include <Rendering/Core/RenderSurface.h>

#include <Rendering/Utils/VulkanUtils.h>

RenderSurface::RenderSurface(VkSurfaceKHR surface, const RenderResourceRef<Instance>& instance)
	: _handle(surface)
	, _instance(instance)
{
	LOG_DEBUG("> Surface created.");
}
