#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>
#include <nfd.h>

#include <UI/WindowSubsystem.h>
#include <Rendering/Utils/VulkanValidation.h>
#include <Rendering/Utils/VulkanUtils.h>

#if PLATFORM_MAC
	constexpr std::array RequiredExtensions = {
		VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME
	};
#endif

void WindowSubsystem::Initialize(const SubsystemsCollection& collection)
{
	// Initialize SDL, we only need graphics support.
	if (SDL_Init(SDL_INIT_VIDEO) != 0)
	{
		LOG_ERROR("SDLError: {}\n", SDL_GetError());

		return;
	}

	// Then initialize NFDe.
	if (NFD_Init() != NFD_OKAY)
	{
		LOG_ERROR("Failed to initialize NFD.\n");

		return;
	}

	SDL_SetWindowMinimumSize(_window, 400, 400);

	constexpr auto windowFlags = SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY | SDL_WINDOW_MAXIMIZED | SDL_WINDOW_VULKAN;
	_window = SDL_CreateWindow("Cage Modeler", 400, 400, windowFlags);

	if (_window == nullptr)
	{
		LOG_ERROR("SDLError: {}\n", SDL_GetError());

		return;
	}
}

void WindowSubsystem::Deinitialize()
{
	// Deinitialize NFDe first.
	NFD_Quit();

	SDL_DestroyWindow(_window);
	SDL_Quit();
}

std::vector<const char *> WindowSubsystem::GetRequiredInstanceExtensions()
{
	uint32_t extensionsCount = 0;
	const auto extensionsPtr = SDL_Vulkan_GetInstanceExtensions(&extensionsCount);

	uint32_t totalExtensionsCount = extensionsCount;

#if PLATFORM_MAC
	totalExtensionsCount += RequiredExtensions.size();
#endif

	if constexpr (MCD::EnableVulkanValidation)
	{
		++totalExtensionsCount;
	}

	std::vector<const char*> extensions;
	extensions.reserve(totalExtensionsCount);
	extensions.resize(extensionsCount);
	std::copy(extensionsPtr, extensionsPtr + extensionsCount, extensions.begin());

	// On macOS we also need to add the portability extension.
#if PLATFORM_MAC
	extensions.insert(extensions.end(), RequiredExtensions.begin(), RequiredExtensions.end());
#endif

	if constexpr (MCD::EnableVulkanValidation)
	{
		extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
	}

	return extensions;
}

VkSurfaceKHR WindowSubsystem::CreateVulkanSurface(const VkInstance instance) const
{
	// Create a new Vulkan surface using SDL.
	VkSurfaceKHR result = nullptr;
	if (!SDL_Vulkan_CreateSurface(_window, instance, nullptr, &result))
	{
		LOG_ERROR("SDLError: {}\n", SDL_GetError());

		return nullptr;
	}

	return result;
}

VkExtent2D WindowSubsystem::GetDrawableSize() const
{
	// Get the window size from SDL.
	int32_t width = std::numeric_limits<int32_t>::max();
	int32_t height = std::numeric_limits<int32_t>::max();
	SDL_GetWindowSizeInPixels(_window, &width, &height);

	return VkExtent2D { static_cast<uint32_t>(width), static_cast<uint32_t>(height) };
}

VkExtent2D WindowSubsystem::GetWindowSize() const
{
	// Get the window size from SDL.
	int32_t width = std::numeric_limits<int32_t>::max();
	int32_t height = std::numeric_limits<int32_t>::max();
	SDL_GetWindowSize(_window, &width, &height);

	return VkExtent2D { static_cast<uint32_t>(width), static_cast<uint32_t>(height) };
}
